import warnings

import h5py
import numpy as np
import pytest

from cpsl_datasets.hdf5 import Capture, pose_at, static_chain, validate

from synth_h5 import CFG_METRICS, build

VARIANTS = ["v2", "v3", "v4", "unclosed"]


@pytest.fixture(params=VARIANTS)
def cap_path(request, tmp_path):
    return build(tmp_path / ("%s.h5" % request.param), request.param), request.param


def test_validate_clean(cap_path):
    path, _ = cap_path
    assert validate(path) == []


def test_streams_discovered(cap_path):
    path, v = cap_path
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with Capture(path) as c:
            names = set(c.streams())
            assert "tf" not in names and "tf_static" not in names
            assert {"radar_adc", "adc_meta", "radar_pc", "cam_rgb", "radar_adc_empty"} <= names
            assert ("imu" in names) == (v != "v2")
            assert ("cam_info" in names) == (v in ("v4", "unclosed"))
            assert c.version == {"v2": 2, "v3": 3}.get(v, 4)
            assert c.closed_cleanly == (v != "unclosed")
            assert bool(c.warnings) == (v == "unclosed")


def test_unclosed_warns(tmp_path):
    p = build(tmp_path / "u.h5", "unclosed")
    with pytest.warns(UserWarning, match="closed_cleanly"):
        Capture(p).close()


def test_cube_and_empty(tmp_path):
    p = build(tmp_path / "a.h5", "v4")
    with Capture(p) as c:
        s = c["radar_adc"]
        assert len(s) == 3 and s.columns == ["real", "imag"] and s.frame_id == "radar"
        x = s[0]
        assert x.dtype == np.complex64 and x.shape == (2, 4, 3)
        raw = s.read(0, raw=True)
        assert raw.dtype == np.int16 and raw.shape == (2, 2, 4, 3)
        assert np.allclose(x, raw[0] + 1j * raw[1])
        e = c["radar_adc_empty"]
        assert len(e) == 0 and len(e.stamps()) == 0
        with pytest.raises(IndexError):
            e[0]


def test_ragged_meta_images(tmp_path):
    p = build(tmp_path / "a.h5", "v4")
    with Capture(p) as c:
        pc = c["radar_pc"]
        assert [pc[i].shape for i in range(3)] == [(4, 4), (0, 4), (2, 4)]
        assert set(pc.named(0)) == {"x", "y", "z", "vel"} and pc.named(0)["x"].shape == (4,)
        assert pc[-1].shape == (2, 4)
        m = c["adc_meta"][1]
        assert m["frame_index"].shape == () and m["t_s"].shape == (3,) and "stamp_recv_ns" not in m
        assert c["cam_rgb"][0].shape == (5, 6, 3) and c["cam_rgb"][0].dtype == np.uint8
        assert c["cam_mono"][0].shape == (5, 6)
        assert c["depth16"][0].dtype == np.uint16 and c["depth32"][0].dtype == np.float32
        assert c["tfmsg"][1].shape == (3, 7)
        assert c["cmd_vel"].stamps("header").tolist() == [-1, -1, -1]
        od = c["odom"]
        assert od.frame_id == "odom" and od.attrs["child_frame_id"] == "base_link"
        assert od.named(0)["qw"].shape == ()
        assert np.isnan(c["cam_info"][0][-1])


def test_stamps_clock(tmp_path):
    p = build(tmp_path / "a.h5", "v3")
    with Capture(p) as c:
        s = c["imu"]
        assert (s.stamps("recv") > s.stamps("header")).all()
        with pytest.raises(ValueError):
            s.stamps("wall")


def test_radar_cfg(tmp_path):
    with Capture(build(tmp_path / "v2.h5", "v2")) as c:
        assert c.radar_cfg() is None
    with Capture(build(tmp_path / "v3.h5", "v3")) as c:
        cfg = c.radar_cfg()
        assert cfg["status"] == "ok" and cfg["metrics"] == CFG_METRICS


def test_tf(tmp_path):
    p = build(tmp_path / "a.h5", "v4")
    with Capture(p) as c:
        assert c.robot_description.startswith("<robot")
        assert len(c.tf_static()) == 2
        assert list(c.tf_pairs()) == [("odom", "base_link")]
        T = c.static_chain("radar", "base_link")
        assert np.allclose(T[:3, 3], [0.1, 0, 0.2])
        assert np.allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-6)  # 90 deg about z
        with pytest.raises(ValueError):
            c.static_chain("radar", "nowhere")
        # tracked x = 0.5 * index, stamps every 100 ms
        t = c["radar_pc"].stamps("header")[0] + 150_000_000
        assert np.isclose(c.pose_at("odom", "base_link", t)[0, 3], 0.75)
        assert np.isclose(c.pose_at("odom", "base_link", t, False)[0, 3] % 0.5, 0.0)
        # clamped past the end
        assert np.isclose(c.pose_at("odom", "base_link", t + 10**12)[0, 3], 2.0)
        T2 = c.pose_at("odom", "base_link", t) @ T
        assert np.allclose(T2[:3, 3], [0.85, 0, 0.2])
    with Capture(build(tmp_path / "v2.h5", "v2")) as c:
        assert c.tf_static() == [] and c.tf_pairs() == {}
        assert c.robot_description is None


def _mutate(path, fn):
    with h5py.File(path, "r+") as f:
        fn(f)


@pytest.mark.parametrize("name,fn,frag", [
    ("dtype", lambda f: (f.__delitem__("imu/data"),
                         f["imu"].create_dataset("data", data=np.zeros((3, 10), np.float32))), "dtype"),
    ("shape", lambda f: (f.__delitem__("odom/data"),
                         f["odom"].create_dataset("data", data=np.zeros((3, 12)))), "shape"),
    ("stamp_len", lambda f: (f.__delitem__("imu/stamp_header_ns"),
                             f["imu"].create_dataset("stamp_header_ns", data=np.zeros(2, np.int64))),
     "stamp lengths"),
    ("rowcount", lambda f: (f.__delitem__("radar_pc/row_count"),
                            f["radar_pc"].create_dataset("row_count", data=np.array([1, 1, 1], np.int64))),
     "row_count sums"),
    ("columns", lambda f: f["imu"].attrs.__setitem__("columns", np.array(["a"] * 10, dtype=h5py.string_dtype())),
     "columns"),
    ("n_samples", lambda f: f["imu"].attrs.__setitem__("n_samples", 9), "n_samples"),
    ("tf_static", lambda f: (f.__delitem__("tf_static/xyz"),
                f["tf_static"].create_dataset("xyz", data=np.zeros((2, 2)))), "tf_static/xyz"),
    ("root", lambda f: f.attrs.__setitem__("format", "other"), "format"),
    ("notf", lambda f: f.__delitem__("tf_static"), "no /tf_static"),
    ("meta", lambda f: (f.__delitem__("adc_meta/t_s"),), "missing field t_s"),
])
def test_validate_catches(tmp_path, name, fn, frag):
    p = build(tmp_path / "a.h5", "v4")
    _mutate(p, fn)
    probs = validate(p)
    assert any(frag in x for x in probs), probs


def test_validate_unreadable(tmp_path):
    (tmp_path / "x.h5").write_bytes(b"nope")
    assert validate(tmp_path / "x.h5")[0].startswith("cannot open")


def test_no_ros_imports():
    import pathlib
    root = pathlib.Path(__file__).resolve().parents[1] / "cpsl_datasets" / "hdf5"
    for p in root.glob("*.py"):
        t = p.read_text()
        assert "rclpy" not in t and "import ros" not in t
