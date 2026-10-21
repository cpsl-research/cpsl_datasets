"""Time-aware access: match / row_stamps / at_time / window / read / interp, the
capture_mode surfacing and the match= kwarg, on exact synthetic all / sampled
captures (synth_h5.build_all / build_sampled)."""
import numpy as np
import pytest
from synth_h5 import (MS, RADAR_HDR_LAG, T0, TICK_NS, VX, YAW_RATE, build,
                      build_all, build_sampled, radar_recv)

from cpsl_datasets import CpslDS
from cpsl_datasets.cpsl_ds import Match, _slerp
from cpsl_datasets.hdf5 import Capture
from cpsl_datasets.inspect import main as inspect_main

NONE = Match(-1, None, None)


@pytest.fixture
def all_path(tmp_path):
    return build_all(tmp_path / "all.h5")


@pytest.fixture
def ds(all_path):
    with CpslDS(all_path) as d:
        yield d


def yaw_q(yaw):
    return np.array([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)])


def test_synthetic_builders_validate(tmp_path):
    from cpsl_datasets.hdf5 import validate
    assert validate(build_all(tmp_path / "a.h5")) == []
    assert validate(build_sampled(tmp_path / "s.h5")[0]) == []


# --- mode surfacing -----------------------------------------------------------
def test_capture_mode_all(all_path):
    with Capture(all_path) as c:
        assert c.capture_mode == "all" and c.sample_rate_hz == 10.0
        assert {s.mode for s in c.streams().values()} == {"all"}


def test_capture_mode_sampled_and_override(tmp_path):
    p, _, _ = build_sampled(tmp_path / "s.h5")
    with Capture(p) as c:
        assert c.capture_mode == "sampled" and c.sample_rate_hz == 10.0
        st = c.streams()
        assert st["radar_adc"].mode == st["odom"].mode == "sampled"
        assert st["vec3"].mode == "all"  # per-stream override


def test_sample_rate_absent_and_old_formats(tmp_path):
    p, _, _ = build_sampled(tmp_path / "s.h5", rate_attr=False)
    with Capture(p) as c:
        assert c.capture_mode == "sampled" and c.sample_rate_hz is None
    with Capture(build(tmp_path / "v2.h5", "v2")) as c:  # no capture_mode in v2
        assert c.capture_mode is None and c.sample_rate_hz is None
        assert all(s.mode is None for s in c.streams().values())


def test_stream_mode_falls_back_to_root(tmp_path):
    p, _, _ = build_sampled(tmp_path / "s.h5")
    import h5py
    with h5py.File(p, "r+") as f:
        del f["radar_adc"].attrs["mode"]
    with Capture(p) as c:
        assert c["radar_adc"].mode == "sampled"


# --- match ----------------------------------------------------------------------
def test_match_reference_itself(ds):
    assert ds.reference == "radar_adc" and ds.num_frames == 6
    for k in range(6):
        assert ds.match("radar_adc", k) == Match(k, radar_recv(k), 0)


def test_match_previous_recv_dt_is_sample_minus_ref(ds):
    # odom has a hole 1500..2000 ms; frame 2 (1500 ms) -> odom at 1450 ms
    m = ds.match("vehicle_odom", 2)
    assert m == Match(29, T0 + 1450 * MS, -50 * MS)
    assert ds.match("vehicle_odom", 4) == Match(29, T0 + 1450 * MS, -550 * MS)
    assert ds.match("vehicle_odom", 5) == Match(34, T0 + 2250 * MS, 0)  # row index skips the hole
    assert ds.match("vehicle_odom", -1) == ds.match("vehicle_odom", 5)
    assert all(isinstance(v, int) for v in ds.match("vehicle_odom", 2))


def test_match_agrees_with_sample_index_and_stamps(ds):
    for role in ("vehicle_odom", "imu", "chirp_meta", "vector3"):
        for k in range(6):
            m = ds.match(role, k)
            assert m.j == ds.sample_index(role, k)
            if m.j >= 0:
                assert m.t_ns == ds.get_stamps(role)[m.j]


def test_no_sample_yet_is_minus_one_not_last_stamp(ds):
    assert ds.match("vector3", 0) == NONE  # first vec3 at 1100 ms, frame 0 at 1000 ms
    assert ds.sample_index("vector3", 0) == -1
    assert len(ds.row_stamps("vector3", 0)) == 0
    assert ds.get_vector3(0).shape == (0, 4)
    assert ds.match("vector3", 1).j == 1  # 1100 and 1200 ms rows, frame 1 at 1250 ms


def test_chirp_meta_both_clocks_and_nearest(all_path):
    with CpslDS(all_path) as d:  # recv: meta arrives 3 ms after its cube
        assert d.match("chirp_meta", 0) == NONE
        assert d.match("chirp_meta", 3) == Match(2, radar_recv(2) + 3 * MS, -247 * MS)
        assert d.get_chirp_meta(3)["frame_index"] == 2  # the previous frame's meta
    with CpslDS(all_path, match="nearest") as d:
        assert d.match("chirp_meta", 0) == Match(0, radar_recv(0) + 3 * MS, 3 * MS)
        assert d.match("chirp_meta", 3).dt_ns == 3 * MS
        assert d.get_chirp_meta(3)["frame_index"] == 3
    with CpslDS(all_path, clock="header") as d:  # same header: exact
        for k in range(6):
            assert d.match("chirp_meta", k) == Match(k, radar_recv(k) - RADAR_HDR_LAG, 0)
            assert d.get_chirp_meta(k)["frame_index"] == k


def test_match_header_clock_odom(all_path):
    with CpslDS(all_path, clock="header") as d:
        t_ref = radar_recv(0) - RADAR_HDR_LAG  # 890 ms
        assert d.match("radar_adc", 0).t_ns == t_ref
        m = d.match("vehicle_odom", 0)  # odom header = recv - 1 ms; newest <= 890 is row 17 (849)
        assert m == Match(17, T0 + 849 * MS, 849 * MS - 890 * MS)


def test_nearest_ties_go_to_earlier_and_windows_unchanged(all_path):
    with CpslDS(all_path) as prev, CpslDS(all_path, match="nearest") as near:
        assert near.match_mode == "nearest" and prev.match_mode == "previous"
        # frame 3 (1750 ms): odom 1450 and 2050 are both 300 ms away -> earlier
        assert near.match("vehicle_odom", 3) == Match(29, T0 + 1450 * MS, -300 * MS)
        # frame 4 (2000 ms): 1450 is 550 away, 2050 is 50 away
        assert near.match("vehicle_odom", 4) == Match(30, T0 + 2050 * MS, 50 * MS)
        assert prev.match("vehicle_odom", 4).j == 29
        for k in range(6):  # row-window getters ignore match=
            assert np.array_equal(prev.get_vehicle_odom_data(k), near.get_vehicle_odom_data(k))
        # nearest never farther than previous
        for k in range(6):
            a, b = prev.match("imu", k), near.match("imu", k)
            assert abs(b.dt_ns) <= abs(a.dt_ns)


def test_match_kwarg_validated(all_path):
    with pytest.raises(ValueError, match="match must be"):
        CpslDS(all_path, match="next")


def test_match_unsynced_has_no_dt(all_path):
    with CpslDS(all_path, sync=False) as d:
        m = d.match("vehicle_odom", 3)
        assert m.j == 3 and m.t_ns == d.get_stamps("vehicle_odom")[3] and m.dt_ns is None


def test_match_reference_selectable(all_path):
    with CpslDS(all_path, reference="vehicle_odom") as d:
        assert d.reference == "vehicle_odom" and d.num_frames == 69
        assert d.match("radar_adc", 0) == NONE  # radar starts at 1000 ms, odom frame 0 is 0 ms
        m = d.match("radar_adc", 20)  # odom row 20 = 1000 ms
        assert m == Match(0, radar_recv(0), 0)
        assert d.match("vehicle_odom", 7) == Match(7, T0 + 350 * MS, 0)


# --- row_stamps ----------------------------------------------------------------------
def test_row_stamps_match_getter_rows(ds):
    for role, getter in (("vehicle_odom", ds.get_vehicle_odom_data), ("imu", ds.get_imu_full_data),
                         ("vector3", ds.get_vector3)):
        for k in range(6):
            rows = getter(k)
            st = ds.row_stamps(role, k)
            assert st.dtype == np.int64 and len(st) == len(rows)
            assert np.allclose(st * 1e-9, rows[:, 0]) if len(rows) else True
    assert len(ds.row_stamps("vehicle_odom", 4)) == 0  # inside the hole
    assert list(ds.row_stamps("vehicle_odom", 5)) == [T0 + t * MS for t in (2050, 2100, 2150, 2200, 2250)]
    # frame-type roles: the one served sample, or empty
    assert list(ds.row_stamps("radar_adc", 2)) == [radar_recv(2)]
    assert list(ds.row_stamps("chirp_meta", 3)) == [radar_recv(2) + 3 * MS]
    assert len(ds.row_stamps("chirp_meta", 0)) == 0


def test_row_stamps_vicon_and_unsynced(all_path):
    with CpslDS(all_path) as d:
        assert d.vicon_names == ["vicon_box"]
        assert len(d.row_stamps("vicon_box", 1)) == len(d.get_vicon_data(1))
    with CpslDS(all_path, sync=False) as d:
        assert list(d.row_stamps("vehicle_odom", 3)) == [d.get_stamps("vehicle_odom")[3]]


# --- at_time / window ----------------------------------------------------------------
def test_at_time_previous_nearest_tol(ds):
    t = T0 + 1475 * MS
    assert ds.at_time("vehicle_odom", t) == Match(29, T0 + 1450 * MS, -25 * MS)
    assert ds.at_time("vehicle_odom", t, "nearest") == Match(29, T0 + 1450 * MS, -25 * MS)
    t = T0 + 2030 * MS
    assert ds.at_time("vehicle_odom", t).dt_ns == -580 * MS
    assert ds.at_time("vehicle_odom", t, "nearest") == Match(30, T0 + 2050 * MS, 20 * MS)
    assert ds.at_time("vehicle_odom", t, tol_ns=100 * MS) == NONE  # previous too stale
    assert ds.at_time("vehicle_odom", t, "nearest", tol_ns=100 * MS).j == 30
    assert ds.at_time("vehicle_odom", t, "previous", tol_ns=580 * MS).j == 29  # edge inclusive
    # exact stamp, then ties: previous takes the sample at t
    assert ds.at_time("imu", T0 + 40 * MS) == Match(2, T0 + 40 * MS, 0)
    assert ds.at_time("imu", T0 + 50 * MS, "nearest").j == 2  # 10 ms vs 10 ms -> earlier


def test_at_time_empty_cases(ds):
    assert ds.at_time("vehicle_odom", T0 - 1) == NONE  # before the first sample
    assert ds.at_time("vehicle_odom", T0 - 1, "nearest").j == 0  # nearest still finds one
    assert ds.at_time("vehicle_odom", T0 - 1, "nearest", tol_ns=0) == NONE
    assert ds.at_time("vehicle_odom", T0 + 10**13).j == 68  # after the last: the last, stale
    assert ds.at_time("vehicle_odom", T0 + 10**13, tol_ns=MS) == NONE
    assert ds.at_time("lidar", T0 + 2 * 10**9) == NONE  # empty stream
    with pytest.raises(ValueError, match="method"):
        ds.at_time("imu", T0, "linear")
    with pytest.raises(ValueError, match="no radar_pc"):
        ds.at_time("radar_pc", T0)


def test_at_time_header_clock(all_path):
    with CpslDS(all_path, clock="header") as d:
        t = T0 + 899 * MS
        assert d.at_time("chirp_meta", t) == Match(0, T0 + 890 * MS, -9 * MS)


def test_window_half_open_and_ordered(ds):
    w = ds.window("vehicle_odom", T0 + 50 * MS, T0 + 150 * MS)  # (50, 150] -> 100, 150
    assert list(w) == [2, 3]
    assert list(ds.window("vehicle_odom", T0 + 49 * MS, T0 + 100 * MS)) == [1, 2]
    assert list(ds.window("vehicle_odom", None, T0 + 50 * MS)) == [0, 1]
    assert list(ds.window("vehicle_odom", T0 + 1450 * MS, T0 + 2050 * MS)) == [30]  # across the hole
    assert len(ds.window("vehicle_odom", T0 + 1450 * MS, T0 + 2000 * MS)) == 0
    assert len(ds.window("vehicle_odom", T0 + 100 * MS, T0 + 100 * MS)) == 0
    assert len(ds.window("vehicle_odom", T0 + 200 * MS, T0 + 100 * MS)) == 0  # inverted
    assert len(ds.window("lidar", T0, T0 + 10**10)) == 0  # empty stream
    assert ds.window("imu", None, T0 + 10**13).dtype.kind == "i"
    assert len(ds.window("imu", None, T0 + 10**13)) == 200


def test_window_equals_frame_window(ds):
    for k in range(1, 6):
        lo, hi = ds.get_stamps("radar_adc")[k - 1], ds.get_stamps("radar_adc")[k]
        w = ds.window("vehicle_odom", lo, hi)
        assert np.array_equal(ds.get_stamps("vehicle_odom")[w], ds.row_stamps("vehicle_odom", k))


# --- read -----------------------------------------------------------------------------
def test_read_matches_getters(ds):
    for k in range(6):
        for role, getter in (("radar_adc", ds.get_radar_adc_data),):
            assert np.array_equal(ds.read(role, ds.match(role, k).j), getter(k))
    assert ds.read("chirp_meta", 2)["frame_index"] == 2
    j = ds.match("vehicle_odom", 5).j
    row = ds.read("vehicle_odom", j)
    assert row.shape == (1, 14) and row[0, 0] == pytest.approx((ds.get_stamps("vehicle_odom")[j]) * 1e-9)
    assert np.array_equal(row[0], ds.get_vehicle_odom_data(5)[-1])
    assert ds.read("imu", 3).shape == (1, 7) and ds.read("vector3", 0).shape == (1, 4)
    assert ds.read("vicon_box", 4).shape == (1, 7)
    assert np.array_equal(ds.read("vicon_box", 4)[0], ds.streams["vicon_box"]._g["data"][4])


def test_read_no_sample_raises_instead_of_last(ds):
    m = ds.match("vector3", 0)
    assert m.j == -1
    with pytest.raises(IndexError, match="no sample"):
        ds.read("vector3", m.j)
    with pytest.raises(IndexError):
        ds.read("vector3", 10**6)
    with pytest.raises(IndexError):
        ds.read("radar_adc", -1)


def test_read_frame_getters_other_types(tmp_path):
    from synth_h5 import build_getters
    with CpslDS(build_getters(tmp_path / "g.h5"), lidar="lidar") as d:
        j = d.match("lidar", 2).j
        assert np.array_equal(d.read("lidar", j), d.get_lidar_point_cloud_raw(2), equal_nan=True)
        assert np.array_equal(d.read("hand_left", d.match("hand_left", 1).j),
                              d.get_hand_tracking_data(1, "left"))
        ci = d.read("camera_info", d.match("camera_info", 3).j)
        assert ci["K"].shape == (3, 3)
        assert d.read("tf_message", 1).shape == (3, 7)
        assert d.read("vehicle_vel", 2).shape == (1, 3)
        assert d.read("camera_depth", 0).shape == (5, 6)


# --- interp -----------------------------------------------------------------------------
def odom_x(t_ms):
    return VX * t_ms / 1000.0


def test_interp_odom_linear_and_slerp_exact(ds):
    for t_ms in (1275, 20, 3951, 3999 - 100):  # between samples, not in the hole
        if 1450 < t_ms < 2050 or t_ms > 3950:
            continue
        v = ds.interp("vehicle_odom", T0 + t_ms * MS)
        assert v.shape == (13,)
        assert v[0] == pytest.approx(odom_x(t_ms), abs=1e-9)
        assert np.allclose(v[3:7], yaw_q(YAW_RATE * t_ms / 1000.0), atol=1e-6)
        assert v[7] == pytest.approx(VX)
        assert np.linalg.norm(v[3:7]) == pytest.approx(1.0)


def test_interp_exact_sample_and_array(ds):
    d = ds.streams["vehicle_odom"]._g["data"][:]
    t = np.array([T0 + 1250 * MS, T0 + 50 * MS, T0 + 1275 * MS])
    out = ds.interp("vehicle_odom", t)
    assert out.shape == (3, 13)
    assert np.array_equal(out[0], d[25]) and np.array_equal(out[1], d[1])
    assert np.isnan(out).sum() == 0
    assert ds.interp("vehicle_odom", [T0 + 100 * MS]).shape == (1, 13)
    assert ds.interp("vehicle_odom", float(T0 + 1275 * MS))[0] == pytest.approx(odom_x(1275))


def test_interp_nan_outside_data(ds):
    assert np.isnan(ds.interp("vehicle_odom", T0 - 1)).all()
    assert np.isnan(ds.interp("vehicle_odom", T0 + 3951 * MS)).all()  # after the last (3950 ms)
    assert not np.isnan(ds.interp("vehicle_odom", T0 + 3950 * MS)).any()  # last stamp exact
    assert not np.isnan(ds.interp("vehicle_odom", T0)).any()
    out = ds.interp("vehicle_odom", np.array([T0 - MS, T0 + 100 * MS, T0 + 10**13]))
    assert np.isnan(out[0]).all() and np.isnan(out[2]).all() and not np.isnan(out[1]).any()


def test_interp_gap_policy(ds):
    t = T0 + 1750 * MS  # inside the 600 ms hole (1450 -> 2050); default max gap 150 ms
    assert np.isnan(ds.interp("vehicle_odom", t)).all()
    assert np.isnan(ds.interp("vehicle_odom", t, max_gap_ns=599 * MS)).all()
    v = ds.interp("vehicle_odom", t, max_gap_ns=600 * MS)  # bridge it explicitly
    assert v[0] == pytest.approx(odom_x(1750), abs=1e-9)
    assert np.allclose(v[3:7], yaw_q(YAW_RATE * 1.75), atol=1e-6)
    # a tight max_gap also rejects ordinary 50 ms steps
    assert np.isnan(ds.interp("vehicle_odom", T0 + 1275 * MS, max_gap_ns=10 * MS)).all()
    # the exact sample at the gap's edge is still returned
    assert not np.isnan(ds.interp("vehicle_odom", T0 + 1450 * MS)).any()


def test_interp_other_streams(ds):
    t = T0 + 1010 * MS
    v = ds.interp("vicon_box", t)  # 50 Hz, rows at 1000 and 1020
    assert v.shape == (7,) and v[0] == pytest.approx(odom_x(1010), abs=1e-9)
    assert np.allclose(v[3:], yaw_q(YAW_RATE * 1.01), atol=1e-6)
    v = ds.interp("imu", t)  # 10 columns, quaternion first; accel col grows 1 per row
    assert v.shape == (10,) and np.allclose(v[:4], yaw_q(YAW_RATE * 1.01), atol=1e-6)
    assert v[4] == pytest.approx(50.5)
    v = ds.interp("vector3", T0 + 1150 * MS)  # 100 ms rows from 1100
    assert v == pytest.approx([1.15, 2.3, 1.0])
    assert np.isnan(ds.interp("vector3", T0 + 1000 * MS)).all()  # before its first sample
    assert ds.interp("vehicle_vel", T0 + 1275 * MS).shape == (13,)  # Odometry fallback


def test_interp_header_clock(all_path):
    with CpslDS(all_path, clock="header") as d:
        # odom header = recv - 1 ms; the x column is a function of recv
        v = d.interp("vehicle_odom", T0 + 1274 * MS)
        assert v[0] == pytest.approx(odom_x(1275), abs=1e-9)


def test_interp_rejects_non_continuous_role(ds):
    with pytest.raises(ValueError, match="interp"):
        ds.interp("radar_adc", T0)
    with pytest.raises(ValueError, match="no radar_pc"):
        ds.interp("radar_pc", T0)


def test_slerp_90_degrees_and_sign_flip():
    qa, qb = yaw_q(0.0)[None], yaw_q(np.pi / 2)[None]
    half = _slerp(qa, qb, np.array([[0.5]]))
    assert np.allclose(half[0], yaw_q(np.pi / 4), atol=1e-12)
    flipped = _slerp(qa, -qb, np.array([[0.5]]))  # same rotation, opposite sign: short arc
    assert np.allclose(np.abs(flipped[0]), np.abs(yaw_q(np.pi / 4)), atol=1e-12)
    same = _slerp(qa, qa, np.array([[0.3]]))  # degenerate (angle 0) must not divide by zero
    assert np.allclose(same[0], qa[0])
    ends = _slerp(qa, qb, np.array([[0.0], ]))
    assert np.allclose(ends[0], qa[0])


def test_interp_empty_and_single_sample_streams(tmp_path):
    import h5py
    p = build(tmp_path / "e.h5", "v4")
    with CpslDS(p, radar_adc="radar_adc") as d:
        assert d.interp("vehicle_odom", T0 + 10**8).shape == (13,)  # 3 rows @100 ms; fine
    with h5py.File(p, "r+") as f:  # make odom a one-sample stream
        for k in ("stamp_header_ns", "stamp_recv_ns"):
            a = f["odom"][k][:1]
            del f["odom"][k]
            f["odom"][k] = a
        d0 = f["odom"]["data"][:1]
        del f["odom"]["data"]
        f["odom"]["data"] = d0
    with CpslDS(p, radar_adc="radar_adc") as d:
        t0 = int(d.get_stamps("vehicle_odom")[0])
        assert not np.isnan(d.interp("vehicle_odom", t0)).any()  # exact
        assert np.isnan(d.interp("vehicle_odom", t0 + 1)).all()  # nothing to bracket


# --- sampled captures ----------------------------------------------------------------------
@pytest.fixture
def sampled(tmp_path):
    p, ticks, pick = build_sampled(tmp_path / "s.h5")
    with CpslDS(p) as d:
        yield d, ticks, pick


def test_sampled_stamps_are_message_recv_not_ticks(sampled):
    d, ticks, pick = sampled
    assert np.array_equal(d.get_stamps("radar_adc"), pick["radar"])
    assert not set(d.get_stamps("radar_adc")) & set(ticks)  # tick time is never stored
    assert 0 < len(pick["radar"]) < len(ticks) and len(pick["odom"]) > len(pick["radar"])
    assert d.num_frames == len(pick["radar"])


def test_sampled_previous_serves_older_row_nearest_is_closer(tmp_path):
    p, ticks, pick = build_sampled(tmp_path / "s.h5")
    with CpslDS(p) as prev, CpslDS(p, match="nearest") as near:
        dts_p, dts_n = [], []
        for k in range(prev.num_frames):
            a, b = prev.match("imu", k), near.match("imu", k)
            if a.j < 0:
                continue
            assert a.dt_ns <= 0
            assert a.dt_ns >= -(TICK_NS + 20 * MS)
            assert abs(b.dt_ns) <= abs(a.dt_ns)
            dts_p.append(abs(a.dt_ns))
            dts_n.append(abs(b.dt_ns))
        assert sum(dts_n) < sum(dts_p)  # nearest strictly helps somewhere


def test_sampled_missing_rows_and_all_override(sampled):
    d, ticks, pick = sampled
    assert d.get_stamps("vector3").tolist() == pick["vec3"].tolist()  # the mode="all" stream
    assert d.match("vector3", 0) == NONE  # first vec3 row is after the first radar row
    with pytest.raises(IndexError):
        d.read("vector3", d.match("vector3", 0).j)


# --- inspect -----------------------------------------------------------------------------------
def test_inspect_all(all_path, capsys):
    assert inspect_main([str(all_path)]) == 0
    out = capsys.readouterr().out
    assert "capture_mode: all  sample_rate_hz: 10" in out
    assert "mode all" in out and "fill" not in out


def test_inspect_sampled_fill_ratio(tmp_path, capsys):
    p, ticks, pick = build_sampled(tmp_path / "s.h5")
    assert inspect_main([str(p)]) == 0
    out = capsys.readouterr().out
    assert "capture_mode: sampled  sample_rate_hz: 10" in out
    fills = {}
    for line in out.splitlines():
        parts = line.split()
        if "fill" in parts:
            fills[parts[0]] = float(parts[parts.index("fill") + 1])
    assert set(fills) == {"radar_adc", "adc_meta", "odom", "vicon_box", "imu", "lidar"}  # not vec3 (all)
    assert 0.2 < fills["radar_adc"] < 0.6 and fills["odom"] > 0.8
    assert fills["lidar"] == 0.0  # empty stream
    assert all(0 < f <= 1.0 for k, f in fills.items() if k != "lidar")
    assert "vec3               mode all" in out


def test_inspect_sampled_without_rate_and_old_format(tmp_path, capsys):
    p, _, _ = build_sampled(tmp_path / "s.h5", rate_attr=False)
    inspect_main([str(p)])
    out = capsys.readouterr().out
    assert "sample_rate_hz: not recorded" in out and "fill n/a" in out
    inspect_main([str(build(tmp_path / "v2.h5", "v2"))])
    out = capsys.readouterr().out
    assert "capture_mode: unknown" in out and "mode unknown" in out


# --- existing getters are untouched --------------------------------------------------------------
def test_existing_getters_unchanged_on_all(ds):
    assert ds.get_radar_adc_data(1).shape == (2, 4, 3)
    assert ds.get_vehicle_odom_data(5).shape == (5, 14)
    assert ds.get_chirp_meta(0) == {}
    assert ds.get_vehicle_odom_data(0).shape == (20 + 1, 14)  # (-inf, t_ref[0]] holds odom to 1000 ms
