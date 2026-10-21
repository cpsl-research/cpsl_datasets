import hashlib
import json
import os
import subprocess
import sys

import numpy as np
import pytest
from synth_v1 import build_v1

from cpsl_datasets.convert_v1 import ConvertError, convert, main
from cpsl_datasets.hdf5 import Capture, validate

VARIANTS = {
    "full": dict(),
    "ugv_like": dict(include=("pc", "lidar", "camera", "vehicle")),
    "uav_like": dict(include=("cube", "pc", "vehicle", "vicon")),
    "imu_full_only": dict(include=("imu_full_only", "pc")),
    "no_time_folders": dict(include=("cube", "pc", "camera")),
    "digit_boundary": dict(start=9998, n=5),
}


def tree_hash(root):
    out = {}
    for dp, _, fs in os.walk(root):
        for f in fs:
            p = os.path.join(dp, f)
            with open(p, "rb") as fh:
                out[os.path.relpath(p, root)] = hashlib.sha256(fh.read()).hexdigest()
    return out


def conv(tmp_path, name="v1", build=None, **kw):
    tr = build_v1(tmp_path / name, **(build or {}))
    out = tmp_path / "out.h5"
    res = convert(tr.root, out, log=lambda *_: None, **kw)
    return tr, out, res


@pytest.mark.parametrize("variant", list(VARIANTS))
def test_variant_converts_and_validates(tmp_path, variant):
    tr, out, res = conv(tmp_path, build=VARIANTS[variant])
    assert validate(out) == []
    with Capture(out) as c:
        assert c.version == 4 and c.closed_cleanly
        assert c.warnings == []
        st = c.streams()
        for folder, arrs in tr.frames.items():
            n_exp = sum(len(a) for a in arrs) if folder in (
                "imu_data_full", "vehicle_vel", "vehicle_odom") else len(arrs)
            names = [n for n, s in st.items() if s.attrs["legacy_folder"] == folder]
            assert names, folder
            if folder == "hands":
                assert sorted(names) == ["hand_left", "hand_right"]
            assert all(len(st[n]) == n_exp for n in names), folder
    assert res["skipped"] == []


def test_root_attrs(tmp_path):
    tr, out, _ = conv(tmp_path, build=dict(extra_folders=("mystery",)))
    with Capture(out) as c:
        a = c.attrs
        assert list(c.attrs["tf_track"]) == [""]
        assert c.tf_static() == [] and c.tf_pairs() == {}
        assert json.loads(a["converted_from"])["converter_version"]
        assert [x if isinstance(x, str) else x.decode()
                for x in a["v1_skipped_folders"]] == ["mystery"]
        assert "node clock" in a["timestamps"]
        cube = c["radar_adc"]
        assert cube.attrs["radar_cfg_status"] == "not_received"
        assert cube.columns == ["real", "imag"]
        assert (c["radar_adc"].stamps("header") == -1).all()
        assert c["radar_pc"].frame_id == "base_link"


def test_base_frame_option(tmp_path):
    tr, out, _ = conv(tmp_path, base_frame="cpsl_ugv_1/base_footprint")
    with Capture(out) as c:
        assert c["lidar"].frame_id == "cpsl_ugv_1/base_footprint"
        assert c["vehicle_odom"].attrs["child_frame_id"] == "cpsl_ugv_1/base_footprint"


def test_hands_frame_default_and_option(tmp_path):
    tr, out, _ = conv(tmp_path)
    with Capture(out) as c:
        for n in ("hand_left", "hand_right", "hand_single"):
            assert c[n].frame_id == "cpsl_human_movement/leap_link"
        assert c["lidar"].frame_id == "base_link"
    tr, out, _ = conv(tmp_path, name="v1b", hands_frame="leap_x", force=True)
    with Capture(out) as c:
        assert c["hand_left"].frame_id == c["hand_single"].frame_id == "leap_x"


def test_values_round_trip(tmp_path):
    tr, out, _ = conv(tmp_path)
    with Capture(out) as c:
        for i, a in enumerate(tr.frames["radar_0_adc"]):
            assert np.array_equal(c["radar_adc"].read(i), a)       # exact
            assert c["radar_adc"].read(i, raw=True).dtype == np.int16
        for f, sn in (("radar_0_pc", "radar_pc"), ("radar_1_pc", "radar_1_pc"),
                      ("radar_combined_pc", "radar_combined_pc"), ("lidar", "lidar")):
            for i, a in enumerate(tr.frames[f]):
                assert np.array_equal(c[sn].read(i), a)               # exact, incl. empty
        assert len(c["radar_pc"].read(1)) == 0
        for i, a in enumerate(tr.frames["hands"]):
            assert np.array_equal(c["hand_left"].read(i), a[0])
            assert np.array_equal(c["hand_right"].read(i), a[1])
        for i, a in enumerate(tr.frames["hand_single"]):
            assert np.array_equal(c["hand_single"].read(i), a)
        # images
        for i, a in enumerate(tr.frames["camera"]):
            assert np.array_equal(c["camera"].read(i), a)
        assert c["camera"].attrs["encoding"] == "bgr8"
        assert c["camera_npy"].attrs["encoding"] == "bgr8"
        assert c["camera_depth"].attrs["encoding"] == "mono16"
        assert np.array_equal(c["camera_depth"].read(2), tr.frames["depth"][2])
        assert c["leap_left"].attrs["encoding"] == "mono8"
        assert c["leap_right"].attrs["encoding"] == "32FC1"
        assert np.array_equal(c["leap_right"].read(1), tr.frames["leap_images_right"][1])


def test_reorders_and_row_conversions(tmp_path):
    tr, out, _ = conv(tmp_path)
    odom = np.concatenate(tr.frames["vehicle_odom"])
    vel = np.concatenate(tr.frames["vehicle_vel"])
    full = np.concatenate(tr.frames["imu_data_full"])
    with Capture(out) as c:
        o = np.array([c["vehicle_odom"].read(i) for i in range(len(odom))])
        assert np.array_equal(o[:, :3], odom[:, 1:4])
        assert np.array_equal(o[:, 3:7], odom[:, [5, 6, 7, 4]])      # x y z w
        assert np.array_equal(o[:, 7:], odom[:, 8:])
        v = np.array([c["vehicle_vel"].read(i) for i in range(len(vel))])
        assert np.array_equal(v[:, 0], vel[:, 1])
        assert np.array_equal(v[:, 5], vel[:, 2])
        assert (v[:, 1:5] == 0).all()
        im = np.array([c["imu_data_full"].read(i) for i in range(len(full))])
        assert np.isnan(im[:, :4]).all()
        assert np.array_equal(im[:, 4:], full[:, 1:])
        q = np.array([c["imu"].read(i) for i in range(4)])
        want = np.array(tr.frames["imu_data"])[:, [1, 2, 3, 0]]
        assert np.array_equal(q[:, :4], want) and np.isnan(q[:, 4:]).all()
        for i, a in enumerate(tr.frames["vicon_x500_8"]):
            assert np.array_equal(c["vicon_x500_8"].read(i),
                                  np.r_[a[:3], a[4:7], a[3]])
        t = c["vehicle_odom"].stamps()
        assert np.array_equal(t, np.rint(odom[:, 0] * 1e9).astype(np.int64))


def test_frame_stamps(tmp_path):
    tr, out, _ = conv(tmp_path)
    with Capture(out) as c:
        last_t = [a[-1, 0] for a in tr.frames["vehicle_odom"]]
        assert np.array_equal(c["radar_pc"].stamps(),
                              np.rint(np.array(last_t) * 1e9).astype(np.int64))
    tr, out, _ = conv(tmp_path / "b", build=VARIANTS["no_time_folders"], frame_rate=4.0)
    with Capture(out) as c:
        assert np.array_equal(c["radar_pc"].stamps(),
                              (np.arange(4) * 250_000_000).astype(np.int64))
        assert "synthesized" in c.attrs["timestamps"]


def test_numeric_frame_order(tmp_path):
    tr, out, _ = conv(tmp_path, build=VARIANTS["digit_boundary"])
    assert tr.numbers == [9998, 9999, 10000, 10001, 10002]
    with Capture(out) as c:
        for i, a in enumerate(tr.frames["lidar"]):
            assert np.array_equal(c["lidar"].read(i), a)


def test_map_override_and_unknown_folder(tmp_path):
    tr = build_v1(tmp_path / "v1", include=("pc",), extra_folders=("weird_pc_x", "mystery"))
    # weird_pc_x matches no pattern; map it as a radar cloud with 4 columns? its
    # arrays are (2,) so mapping it as 'pose' must fail loudly instead.
    res = convert(tr.root, tmp_path / "o.h5", log=lambda *_: None)
    assert sorted(res["skipped"]) == ["mystery", "weird_pc_x"]
    with pytest.raises(ConvertError):
        convert(tr.root, tmp_path / "o2.h5", {"mystery": "pose"}, log=lambda *_: None)
    assert not (tmp_path / "o2.h5").exists()
    assert not (tmp_path / "o2.h5.partial").exists()
    res = convert(tr.root, tmp_path / "o3.h5", {"mystery": "skip", "weird_pc_x": "skip"},
                  log=lambda *_: None)
    assert sorted(res["skipped"]) == ["mystery", "weird_pc_x"]
    with pytest.raises(ConvertError):
        convert(tr.root, tmp_path / "o4.h5", {"nosuch": "skip"}, log=lambda *_: None)


def test_map_overrides_pattern(tmp_path):
    tr = build_v1(tmp_path / "v1", include=("pc",))
    res = convert(tr.root, tmp_path / "o.h5", {"radar_0_pc": "lidar"}, log=lambda *_: None)
    with Capture(tmp_path / "o.h5") as c:
        assert c["radar_pc"].columns[-1] == "intensity"
        assert c["radar_1_pc"].columns[-1] == "vel"


def test_cube_never_rounds(tmp_path):
    tr = build_v1(tmp_path / "v1", include=("cube",))
    p = tr.root / "radar_0_adc" / "frame_10002.npy"
    a = np.load(p)
    np.save(p, a + 0.5)
    with pytest.raises(ConvertError, match="non-integer"):
        convert(tr.root, tmp_path / "o.h5", log=lambda *_: None)
    np.save(p, a * 100.0)            # integer but beyond int16
    with pytest.raises(ConvertError, match="int16"):
        convert(tr.root, tmp_path / "o.h5", log=lambda *_: None)
    assert not (tmp_path / "o.h5").exists()


def test_refuses_unsafe_outputs(tmp_path):
    tr = build_v1(tmp_path / "v1", include=("pc",))
    before = tree_hash(tr.root)
    with pytest.raises(ConvertError, match="inside"):
        convert(tr.root, tr.root / "out.h5", log=lambda *_: None)
    with pytest.raises(ConvertError, match="inside"):
        convert(tr.root, tr.root / "radar_0_pc" / "out.h5", log=lambda *_: None)
    out = tmp_path / "o.h5"
    out.write_bytes(b"keep")
    with pytest.raises(ConvertError, match="exists"):
        convert(tr.root, out, log=lambda *_: None)
    assert out.read_bytes() == b"keep"
    convert(tr.root, out, force=True, log=lambda *_: None)
    assert validate(out) == []
    assert tree_hash(tr.root) == before


@pytest.mark.parametrize("variant", ["full", "ugv_like", "uav_like"])
def test_source_unchanged(tmp_path, variant):
    tr = build_v1(tmp_path / "v1", **VARIANTS[variant], extra_folders=("mystery",))
    before = tree_hash(tr.root)
    assert len(before) > 10
    convert(tr.root, tmp_path / "o.h5", log=lambda *_: None)
    assert tree_hash(tr.root) == before
    assert not any(p.name.endswith(".partial") for p in tmp_path.iterdir())


def test_source_read_only_dir(tmp_path):
    """A read-only source (like the real zips' extracts) converts fine."""
    tr = build_v1(tmp_path / "v1", include=("pc", "vehicle"))
    for dp, dn, fn in os.walk(tr.root):
        for f in fn:
            os.chmod(os.path.join(dp, f), 0o444)
        os.chmod(dp, 0o555)
    try:
        convert(tr.root, tmp_path / "o.h5", log=lambda *_: None)
    finally:
        for dp, dn, fn in os.walk(tr.root):
            os.chmod(dp, 0o755)
    assert validate(tmp_path / "o.h5") == []


def test_cli_and_inspect(tmp_path):
    tr = build_v1(tmp_path / "v1", extra_folders=("mystery",))
    out = tmp_path / "o.h5"
    r = subprocess.run([sys.executable, "-m", "cpsl_datasets.convert_v1", str(tr.root),
                        str(out), "--map", "mystery=skip"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert "skipped folder" in r.stdout and "mystery" in r.stdout
    r = subprocess.run([sys.executable, "-m", "cpsl_datasets.inspect", "--validate", str(out)],
                       capture_output=True, text=True)
    assert r.returncode == 0 and "no problems" in r.stdout
    # refuses an existing output without --force, exit 1
    assert main([str(tr.root), str(out)]) == 1


CONVERTER = os.environ.get("CPSL_HDF5_TO_LEGACY")


@pytest.mark.skipif(not CONVERTER, reason="set CPSL_HDF5_TO_LEGACY to Sensors' hdf5_to_legacy.py")
def test_hdf5_to_legacy_round_trip(tmp_path):
    tr, out, _ = conv(tmp_path, build=dict(include=("cube", "pc", "vehicle")))
    back = tmp_path / "back"
    r = subprocess.run([sys.executable, CONVERTER, str(out), str(back),
                        "--reference", "radar_adc"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    for folder in ("radar_0_adc", "radar_0_pc"):
        for k, a in enumerate(tr.frames[folder]):
            got = np.load(back / folder / ("frame_%d.npy" % k))
            assert np.array_equal(got, a), (folder, k)


def test_stream_names_follow_recorder_configs():
    from cpsl_datasets.convert_v1 import stream_name
    want = {"radar_0_adc": "radar_adc", "radar_0_pc": "radar_pc",
            "front_radar_adc": "radar_front_adc", "down_radar_pc": "radar_down_pc",
            "depth": "camera_depth", "imu_data": "imu",
            "leap_images_left": "leap_left", "leap_images_right": "leap_right",
            "camera": "camera", "lidar": "lidar", "vehicle_odom": "vehicle_odom",
            "radar_1_pc": "radar_1_pc", "imu_data_full": "imu_data_full",
            "vehicle_vel": "vehicle_vel", "vicon_x500_8": "vicon_x500_8"}
    for folder, name in want.items():
        assert stream_name(folder) == name, folder
