"""CpslDS v2 over synthetic HDF5 captures (tests/synth_h5.py)."""
import warnings

import h5py
import numpy as np
import pytest

from cpsl_datasets import CpslDS
from cpsl_datasets.hdf5 import tf as tfm

from synth_h5 import CFG_METRICS, T0, Builder, _strs, build

MS = 1_000_000
ODOM_COLS = "x y z qx qy qz qw vx vy vz wx wy wz".split()


@pytest.fixture
def v4(tmp_path):
    return build(tmp_path / "v4.h5", "v4")


def mixed(path, odom_n=20, with_static=True):
    """radar_adc 4 @100 ms, lidar 4 @100 ms, odom @20 ms, camera 2 @250 ms."""
    b = Builder(path, 4)
    rng = b.rng
    b.stream("adc", "raw_radar_msgs/msg/ADCDataCube", 4,
             attrs={"frame_id": "radar", "columns": ["real", "imag"]},
             fields={"data": rng.integers(-9, 9, (4, 2, 2, 3, 5)).astype(np.int16)})
    rc = [3, 2, 0, 4]
    pts = rng.normal(size=(sum(rc), 4)).astype(np.float32) * 0.1
    b.stream("os", "sensor_msgs/msg/PointCloud2", 4,
             attrs={"frame_id": "lidar", "columns": ["x", "y", "z", "intensity"]},
             fields={"data": pts}, ragged=rc)
    od = np.zeros((odom_n, 13))
    od[:, 0] = np.arange(odom_n)  # x = row number
    od[:, 6] = 1.0
    b.stream("vehicle", "nav_msgs/msg/Odometry", odom_n, rate_ns=20 * MS,
             attrs={"frame_id": "odom", "child_frame_id": "base_link", "columns": ODOM_COLS},
             fields={"data": od})
    b.stream("cam", "sensor_msgs/msg/Image", 2, rate_ns=250 * MS,
             attrs={"frame_id": "camera", "encoding": "bgr8"},
             fields={"data": np.arange(2 * 4 * 5 * 3, dtype=np.uint8).reshape(2, 4, 5, 3)})
    if with_static:
        sg = b.f.create_group("tf_static")
        sg["parent"] = _strs(["base_link", "base_link"])
        sg["child"] = _strs(["lidar", "radar"])
        sg["xyz"] = np.array([[1.0, 0, 0], [0, 0, 0.5]])
        sg["quat_xyzw"] = np.array([[0, 0, 0.7071068, 0.7071068], [0, 0, 0, 1.0]])
    b.close()
    return path


def oracle(t_stream, t):
    """Newest index with stamp <= t (later row on ties), -1 if none."""
    best = -1
    for k, v in enumerate(t_stream):
        if v <= t and (best < 0 or v >= t_stream[best]):
            best = k
    return best


# --- import, auto-detect --------------------------------------------------
def test_reexported():
    import cpsl_datasets
    assert cpsl_datasets.CpslDS is CpslDS


def test_autodetect_clash_raises(v4):
    # radar_adc and radar_adc_empty are both ADCDataCube
    with pytest.raises(ValueError, match="radar_adc_empty"):
        CpslDS(v4)


def test_autodetect_roles(v4):
    with CpslDS(v4, radar_adc="radar_adc") as ds:
        assert ds.roles == {"radar_adc": "radar_adc", "radar_pc": "radar_pc",
                            "camera": "cam_rgb", "vehicle_odom": "odom"}
        assert ds.reference == "radar_adc" and ds.num_frames == 3
        with pytest.raises(ValueError, match="no lidar"):
            ds.get_lidar_point_cloud(0)


def test_role_off_and_unknown_name(v4):
    with CpslDS(v4, radar_adc=False, camera="") as ds:
        assert "radar_adc" not in ds.roles and "camera" not in ds.roles
        assert ds.reference == "radar_pc"
    with pytest.raises(KeyError, match="nope"):
        CpslDS(v4, radar_adc="nope")


def test_mixed_autodetect_lidar_vs_radar_pc(tmp_path):
    with CpslDS(mixed(tmp_path / "m.h5")) as ds:
        assert ds.roles == {"radar_adc": "adc", "lidar": "os", "camera": "cam",
                            "vehicle_odom": "vehicle"}


# --- sync on / off ----------------------------------------------------------
def test_sync_matches_oracle_recv(v4):
    with CpslDS(v4, radar_adc="radar_adc", camera="cam_mono") as ds:
        t_ref = ds.get_stamps("radar_adc")
        assert ds.clock == "recv"
        with h5py.File(v4) as f:
            assert (t_ref == f["radar_adc/stamp_recv_ns"][:]).all()
            for i in range(ds.num_frames):
                assert ds.sample_index("radar_adc", i) == i
                for role, name in (("radar_pc", "radar_pc"), ("camera", "cam_mono")):
                    exp = oracle(f[name]["stamp_recv_ns"][:], t_ref[i])
                    assert ds.sample_index(role, i) == exp
                    got = ds.get_radar_point_cloud(i) if role == "radar_pc" \
                        else ds.get_camera_frame(i)
                    if exp < 0:
                        assert got.size == 0
                    elif role == "camera":
                        assert (got == f["cam_mono/data"][exp]).all()
            d = f["radar_adc/data"][1].astype(np.float32)
        adc = ds.get_radar_adc_data(1)
        assert adc.shape == (2, 4, 3) and np.iscomplexobj(adc)
        assert np.allclose(adc, d[0] + 1j * d[1])


def test_sync_header_clock_deterministic(tmp_path):
    with CpslDS(mixed(tmp_path / "m.h5"), clock="header") as ds:
        assert ds.reference == "radar_adc" and ds.num_frames == 4
        assert [ds.sample_index("vehicle_odom", i) for i in range(4)] == [0, 5, 10, 15]
        assert [ds.sample_index("camera", i) for i in range(4)] == [0, 0, 0, 1]
        assert [ds.sample_index("lidar", i) for i in range(4)] == [0, 1, 2, 3]
        assert ds.get_camera_frame(3).shape == (4, 5, 3)
        assert ds.camera_encoding == "bgr8"


def test_sync_off_raw_indices(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p, sync=False) as ds, h5py.File(p) as f:
        assert ds.num_frames == 4
        assert (ds.get_camera_frame(1) == f["cam/data"][1]).all()
        assert ds.get_vehicle_odom_data(17).shape == (1, 14)
        assert ds.get_vehicle_odom_data(17)[0, 1] == 17
        with pytest.raises(IndexError):
            ds.get_camera_frame(2)


def test_reference_kwarg(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p, reference="camera", clock="header") as ds:
        assert ds.num_frames == 2
        assert [ds.sample_index("radar_adc", i) for i in range(2)] == [0, 2]
    with pytest.raises(ValueError, match="reference"):
        CpslDS(p, reference="radar_pc")


# --- odom windows -------------------------------------------------------------
def test_odom_window_header(tmp_path):
    with CpslDS(mixed(tmp_path / "m.h5"), clock="header") as ds:
        w = [ds.get_vehicle_odom_data(i) for i in range(4)]
        # (-inf, t0], then (t_{i-1}, t_i]: rows 0 | 1..5 | 6..10 | 11..15
        assert [list(x[:, 1].astype(int)) for x in w] == [
            [0], [1, 2, 3, 4, 5], [6, 7, 8, 9, 10], [11, 12, 13, 14, 15]]
        assert w[1].shape == (5, 14)
        assert np.allclose(w[1][:, 0], (T0 + np.arange(1, 6) * 20 * MS) * 1e-9)
        assert (w[1][:, 7] == 1.0).all()  # qw is column 6 of the stored row: x-first


def test_odom_window_partition_recv(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p) as ds, h5py.File(p) as f:
        to, tr = f["vehicle/stamp_recv_ns"][:], f["adc/stamp_recv_ns"][:]
        got = np.concatenate([ds.get_vehicle_odom_data(i)[:, 1] for i in range(4)])
        exp = [k for k in range(len(to)) if to[k] <= tr[-1]]
        assert sorted(got.astype(int)) == exp  # each row in exactly one frame
        for i in range(1, 4):
            rows = ds.get_vehicle_odom_data(i)[:, 1].astype(int)
            assert all(tr[i - 1] < to[k] <= tr[i] for k in rows)


def test_odom_only_capture_has_no_reference(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p, radar_adc=False, lidar=False, camera=False) as ds:
        assert ds.reference is None and ds.num_frames == 0
        with pytest.raises(RuntimeError, match="reference"):
            ds.get_vehicle_odom_data(0)
    with CpslDS(p, radar_adc=False, lidar=False, camera=False,
                reference="vehicle_odom") as ds:
        assert ds.num_frames == 20
        assert ds.get_vehicle_odom_data(3)[:, 1].tolist() == [3.0]


# --- empty stream -------------------------------------------------------------
def test_empty_stream_role(v4):
    with CpslDS(v4, radar_adc="radar_adc_empty") as ds:
        assert ds.reference == "radar_pc"  # empty radar_adc is skipped
        assert ds.get_radar_adc_data(0).shape == (0, 0, 0)
        assert len(ds.get_stamps("radar_adc")) == 0
    with CpslDS(v4, radar_adc="radar_adc_empty", sync=False) as ds:
        with pytest.raises(IndexError):
            ds.get_radar_adc_data(0)


def test_empty_cloud_sample(v4):
    with CpslDS(v4, radar_adc="radar_adc", sync=False) as ds:
        assert ds.get_radar_point_cloud(1).shape == (0, 4)  # row_count 0
        assert ds.get_radar_point_cloud(0).shape == (4, 4)


def test_before_first_sample_is_empty(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with h5py.File(p, "a") as f:
        f["cam/stamp_header_ns"][...] += 150 * MS  # camera starts after frame 1
    with CpslDS(p, clock="header") as ds:
        assert ds.sample_index("camera", 0) == -1
        assert ds.get_camera_frame(0).size == 0
        assert ds.get_camera_frame(2).shape == (4, 5, 3)


# --- clock ---------------------------------------------------------------------
def test_header_clock_rejects_unstamped(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with h5py.File(p, "a") as f:
        f["vehicle/stamp_header_ns"][...] = -1
    with pytest.raises(ValueError, match="header"):
        CpslDS(p, clock="header")
    with pytest.raises(ValueError):
        CpslDS(p, clock="sensor")


# --- frame ---------------------------------------------------------------------
@pytest.mark.parametrize("variant", ["v2", "v3"])
def test_frame_needs_tf_static(tmp_path, variant):
    p = build(tmp_path / (variant + ".h5"), variant)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with pytest.raises(ValueError, match="tf_static"):
            CpslDS(p, radar_adc="radar_adc", frame="base_link")
        with CpslDS(p, radar_adc="radar_adc") as ds:  # sensor frame still fine
            assert ds.get_radar_point_cloud(0).shape == (4, 4)
            with pytest.raises(ValueError, match="v4"):
                ds.get_transform("base_link", "radar")


def test_frame_no_static_path(v4):
    with pytest.raises(ValueError, match="no static path"):
        CpslDS(v4, radar_adc="radar_adc", frame="map")


def test_frame_base_applies_static_chain(v4):
    with CpslDS(v4, radar_adc="radar_adc", sync=False) as raw, \
            CpslDS(v4, radar_adc="radar_adc", sync=False, frame="base_link") as base, \
            h5py.File(v4) as f:
        s, b = raw.get_radar_point_cloud(0), base.get_radar_point_cloud(0)
        T = tfm.static_chain(f, "radar", "base_link")
        exp = s[:, :3].astype(np.float64) @ T[:3, :3].T + T[:3, 3]
        assert np.allclose(b[:, :3], exp, atol=1e-5)
        assert (b[:, 3] == s[:, 3]).all()  # vel untouched
        assert not np.allclose(b[:, :3], s[:, :3])


def test_lidar_filter_and_frame(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p, sync=False) as ds, CpslDS(p, sync=False, frame="base_link") as bs:
        raw = ds.get_lidar_point_cloud_raw(0)
        assert raw.shape == (3, 4)
        z = raw[:, 2]
        assert ds.get_lidar_point_cloud(0).shape == (int(((z > -0.2) & (z < 0.1)).sum()), 2)
        b = bs.get_lidar_point_cloud_raw(0)
        # +90 deg yaw then +1 m x: (x, y) -> (1 - y, x)
        assert np.allclose(b[:, 0], 1 - raw[:, 1], atol=1e-5)
        assert np.allclose(b[:, 1], raw[:, 0], atol=1e-5)
        assert ds.get_lidar_point_cloud_raw(2).shape == (0, 4)


# --- cfg, transforms ---------------------------------------------------------------
def test_radar_cfg(v4, tmp_path):
    with CpslDS(v4, radar_adc="radar_adc") as ds:
        assert ds.get_radar_cfg()["metrics"] == CFG_METRICS
    p = build(tmp_path / "v2.h5", "v2")
    with CpslDS(p, radar_adc="radar_adc") as ds:
        assert ds.get_radar_cfg() is None


def test_get_transform(v4):
    with CpslDS(v4, radar_adc="radar_adc") as ds, h5py.File(v4) as f:
        T_br = ds.get_transform("base_link", "radar")
        assert np.allclose(T_br, tfm.static_chain(f, "radar", "base_link"))
        T_cr = ds.get_transform("camera", "radar")
        T_bc = tfm.static_chain(f, "camera", "base_link")
        assert np.allclose(T_cr, np.linalg.inv(T_bc) @ T_br)
        with pytest.raises(ValueError, match="tracked"):
            ds.get_transform("odom", "radar")
        g = f["tf/odom__base_link"]
        t = int(ds.get_stamps("radar_adc")[1])
        shim = {"stamp_header_ns": g["stamp_recv_ns"][:], "data": g["data"][:]}
        exp = tfm.pose_at(shim, t) @ T_br
        assert np.allclose(ds.get_transform("odom", "radar", idx=1), exp)
        assert np.allclose(ds.get_transform("odom", "radar", t_ns=t), exp)
        with pytest.raises(ValueError, match="no tf path"):
            ds.get_transform("map", "radar", idx=0)
