"""CpslDS secondary getters (vicon, hands, Leap, full lidar columns, IMU, vel, ...)
over synthetic captures. Synth only: no local capture holds Vicon, hands or Leap."""
import warnings

import numpy as np
import pytest

from cpsl_datasets import CpslDS
from cpsl_datasets.cpsl_ds import TYPE_GETTERS
import importlib
from synth_h5 import QUATS, build, build_getters, build_human_movement


@pytest.fixture
def g(tmp_path):
    return build_getters(tmp_path / "g.h5")


def test_vicon_dict_and_name_selection(g):
    with CpslDS(g) as ds:
        assert ds.vicon_names == ["vicon_a", "vicon_b"]
        assert ds.vicon_frames["vicon_a"] == ("vicon_world", "seg_a")
        d = ds.get_vicon_data(1)
        assert set(d) == {"vicon_a", "vicon_b"}
        assert d["vicon_a"].shape[1] == 7 and len(d["vicon_a"]) >= 1
        a = ds.get_vicon_data(1, name="seg_a")  # by child_frame_id
        np.testing.assert_array_equal(a, d["vicon_a"])
        with pytest.raises(KeyError):
            ds.get_vicon_data(1, name="nope")


def test_vicon_quaternion_round_trip_and_wxyz(g):
    with CpslDS(g, sync=False) as ds:
        for k in (0, 5, 7):
            xyzw = ds.get_vicon_data(k, name="vicon_a")[0]
            np.testing.assert_allclose(xyzw[3:], QUATS[k % 4])  # stored order, x first
            wxyz = ds.get_vicon_data(k, name="vicon_a", wxyz=True)[0]
            np.testing.assert_allclose(wxyz[:3], xyzw[:3])
            np.testing.assert_allclose(wxyz[3:], [QUATS[k % 4][3], *QUATS[k % 4][:3]])
            np.testing.assert_allclose(wxyz[[4, 5, 6, 3]], xyzw[3:])  # back again


def test_vicon_single_stream_is_an_array(tmp_path):
    p = build_human_movement(tmp_path / "hm.h5")
    with CpslDS(p) as ds:
        assert ds.vicon_names == ["vicon_box"]
        assert isinstance(ds.get_vicon_data(2), np.ndarray)
        assert ds.vicon_frames["vicon_box"][1] == "box"


def test_vicon_absent(tmp_path):
    p = build(tmp_path / "v3.h5", "v3")
    with CpslDS(p, radar_adc="radar_adc") as ds:
        with pytest.raises(ValueError, match="vicon"):
            ds.get_vicon_data(0)


def test_hands_single_streams_and_absent_flag(tmp_path):
    p = build_human_movement(tmp_path / "hm.h5")
    with CpslDS(p, sync=False) as ds:  # raw sample index: deterministic
        with pytest.raises(ValueError, match="hand='left'"):
            ds.get_hand_tracking_data(0)
        r = ds.get_hand_tracking_data(1, hand="right")
        assert r.shape == (21, 3) and ds.hand_present(1, "right")
        left = ds.get_hand_tracking_data(1, hand="left")  # absent: zero rows
        assert left.shape == (21, 3) and not left.any() and not ds.hand_present(1, "left")
        assert ds.hand_present(0, "left")
        with pytest.raises(ValueError, match="hand must be"):
            ds.get_hand_tracking_data(0, hand="middle")
    with CpslDS(p, reference="hand_left") as ds:  # synced to a hand stream
        assert not ds.hand_present(1, "left") and ds.hand_present(2, "right")


def test_hands_v1_style_single_stream(tmp_path):
    import h5py
    p = build_human_movement(tmp_path / "hm.h5")
    with h5py.File(p, "r+") as f:
        f.move("hand_right", "glove")
        del f["hand_left"]
        f["glove"].attrs["frame_id"] = "world"
    with CpslDS(p, sync=False) as ds:
        assert ds.get_hand_tracking_data(0).shape == (21, 3)
        assert ds.hand_present(0)


def test_leap_depth_and_camera_images(g):
    with CpslDS(g, sync=False) as ds:
        for side in ("left", "right"):
            im = getattr(ds, "get_leap_motion_image_frame_" + side)(1)
            assert im.shape == (4, 6) and im.dtype == np.uint8
        assert ds.get_camera_depth_frame(0).dtype == np.uint16
    with CpslDS(g) as ds:  # synced: newest at or before the radar frame
        assert ds.get_leap_motion_image_frame_left(3).shape == (4, 6)


def test_leap_absent_raises(v4_path):
    with CpslDS(v4_path, radar_adc="radar_adc") as ds:
        with pytest.raises(ValueError, match="no leap_left"):
            ds.get_leap_motion_image_frame_left(0)


@pytest.fixture
def v4_path(tmp_path):
    return build(tmp_path / "v4.h5", "v4")


def test_full_lidar_columns_and_invalid(g):
    with CpslDS(g) as ds:
        c = ds.get_point_cloud_columns(0)
        assert list(c) == ["x", "y", "z", "intensity", "t", "reflectivity", "ring",
                           "ambient", "range"]
        assert all(len(v) == 3 for v in c.values())
        assert np.isnan(c["x"][1])
        d = ds.get_point_cloud_columns(0, drop_invalid=True)
        assert all(len(v) == 1 for v in d.values())  # NaN and zero point dropped
        np.testing.assert_array_equal(d["ring"], c["ring"][:1])
        with pytest.raises(ValueError, match="role must be"):
            ds.get_point_cloud_columns(0, role="hand")


def test_imu_vel_odom_style(g):
    with CpslDS(g, vehicle_odom=False) as ds:
        full = np.vstack([ds.get_imu_full_data(i) for i in range(len(ds))])
        assert full.shape[1] == 7
        assert (np.diff(full[:, 0]) > 0).all()
        np.testing.assert_allclose(full[0, 1:], np.arange(6) * 0.1 + 0)  # wx..az of row 0
        v = np.vstack([ds.get_vehicle_vel_data(i) for i in range(len(ds))])
        assert v.shape[1] == 3
        np.testing.assert_allclose(v[0, 1:], [0.0, 5.0])  # vx = col 0, wz = col 5
        assert ds.roles["vehicle_vel"] == "vehicle_vel"  # the name wins over cmd_vel
        yaw = ds.get_imu_orientation_rad(0)
        assert yaw == pytest.approx(0.6)
        assert ds.get_vector3(1).shape[1] == 4


def test_vel_from_odometry_fallback(tmp_path):
    from test_cpsl_ds import mixed
    with CpslDS(mixed(tmp_path / "m.h5")) as ds:
        v = ds.get_vehicle_vel_data(1)
        o = ds.get_vehicle_odom_data(1)
        assert v.shape == (len(o), 3)
        np.testing.assert_allclose(v[:, 1], o[:, 8])  # vx
        np.testing.assert_allclose(v[:, 2], o[:, 13])  # wz


def test_imu_window_shape_with_radar_reference(g):
    with CpslDS(g, reference="radar_adc") as ds:
        assert ds.get_imu_full_data(0).shape[1] == 7


def test_camera_info_chirp_meta_tf_message_radar_data(v4_path):
    with CpslDS(v4_path, radar_adc="radar_adc") as ds:
        ci = ds.get_camera_info(0)
        assert ci["K"].shape == (3, 3) and ci["P"].shape == (3, 4)
        assert ci["D"].shape == (5,) and ci["distortion_model"] == "plumb_bob"
        assert ci["width"] == int(round(ci["width"])) and ci["frame_id"] == "camera"
        assert "frame_index" in ds.get_chirp_meta(1)
        assert ds.get_tf_message(0).shape[1] == 7
        with pytest.warns(DeprecationWarning):
            assert ds.get_radar_data(0).shape[1] == 4
        with pytest.warns(DeprecationWarning):
            ds2 = CpslDS(v4_path, radar_adc="radar_adc", radar_pc=False)
            assert ds2.get_radar_data(0).ndim == 3
            ds2.close()


def test_explicit_wrong_type_raises(g):
    with pytest.raises(ValueError, match="needs"):
        CpslDS(g, imu="lidar")


def test_every_decoder_type_has_a_getter():
    V = importlib.import_module("cpsl_datasets.hdf5.validate")
    types = set(V.ROW_TYPES) | set(V.RAGGED_TYPES) | {"ADCDataCube", "ADCChirpMeta", "Image"}
    assert not types - set(TYPE_GETTERS), "no getter for %s" % sorted(types - set(TYPE_GETTERS))
    for t, names in TYPE_GETTERS.items():
        for n in names:
            assert callable(getattr(CpslDS, n)), (t, n)
