"""Timing-grid reference (reference="grid:<hz>"): span, spacing, matching, defaults,
and a regression that sensor-reference captures are unchanged (b210-21 Step 1)."""
import hashlib
import warnings

import h5py
import numpy as np
import pytest
from synth_h5 import build_all, build_getters, build_sampled
from test_cpsl_ds import mixed

from cpsl_datasets import CpslDS


@pytest.fixture
def all_path(tmp_path):
    return build_all(tmp_path / "all.h5")


def span(ds):
    """Intersection of the bound non-empty streams' stamp ranges (camera_info excluded)."""
    ts = [t for r, t in ds._t.items() if r != "camera_info" and len(t)]
    return max(int(t.min()) for t in ts), min(int(t.max()) for t in ts)


def test_grid_count_spacing_span(all_path):
    for hz in (10, 3, 0.5, 20):
        with CpslDS(all_path, reference="grid:%g" % hz) as ds:
            t0, t1 = span(ds)
            period = round(1e9 / hz)
            assert ds.reference == "grid" and ds.grid_hz == hz
            assert ds.num_frames == len(ds) == (t1 - t0) // period + 1
            ts = np.array([ds._ref_time(k)[1] for k in range(len(ds))])
            assert ts[0] == t0 and (np.diff(ts) == period).all()  # integer ns, no drift
            assert ts[-1] <= t1 < ts[-1] + period
            assert set(ds.roles) >= {"radar_adc", "vehicle_odom"}  # roles unchanged
    with CpslDS(all_path, reference="grid:10") as ds:
        assert ds.num_frames == 12  # [1.1 s, 2.25 s] at 10 Hz
    with CpslDS(all_path, reference="grid:3") as ds:
        assert ds.num_frames == 4 and ds._ref_time(1)[1] - ds._ref_time(0)[1] == 333_333_333


def test_bare_grid_is_10hz(all_path):
    with CpslDS(all_path, reference="grid") as a, CpslDS(all_path, reference="grid:10") as b:
        assert a.grid_hz == 10.0 and a.num_frames == b.num_frames
        assert (a._ref_t == b._ref_t).all()


@pytest.mark.parametrize("bad", ["grid:0", "grid:-2", "grid:abc", "grid:", "grid:nan",
                                 "grid:inf", "grid:1e12"])
def test_bad_rate(all_path, bad):
    with pytest.raises(ValueError, match="grid"):
        CpslDS(all_path, reference=bad)


def test_grid_matching(all_path):
    for mode in ("previous", "nearest"):
        with CpslDS(all_path, reference="grid:10", match=mode) as ds:
            for k in range(len(ds)):
                tg = ds._ref_time(k)[1]
                for role in ("radar_adc", "vehicle_odom", "imu", "vec3"):
                    if role not in ds.roles:
                        continue
                    t = ds._t[role]
                    m = ds.match(role, k)
                    prev = t[t <= tg].max() if (t <= tg).any() else None
                    if mode == "previous":
                        assert (m.t_ns == prev) if prev is not None else m.j == -1
                        assert m.dt_ns <= 0
                    else:
                        best = t[np.argmin(np.abs(t - tg))]
                        assert abs(m.dt_ns) == abs(int(best) - tg)
                    assert m.dt_ns == m.t_ns - tg
                    assert int(ds.sample_index(role, k)) == m.j
    with CpslDS(all_path, reference="grid:10", match="nearest") as n, \
            CpslDS(all_path, reference="grid:10") as p:
        # vec3 starts 1.1 s in: at frame 0 (t = 1.1 s) previous and nearest both hit row 0
        assert p.match("radar_adc", 0).t_ns <= p._ref_time(0)[1]
        assert any(n.match("radar_adc", k).dt_ns > 0 for k in range(len(n)))  # nearest looks ahead


def test_grid_windows_partition_rows_once(all_path):
    with CpslDS(all_path, reference="grid:10") as ds:
        t = ds._t["vehicle_odom"]
        wins = [ds._window("vehicle_odom", k) for k in range(len(ds))]
        cat = np.concatenate(wins)
        last = ds._ref_time(len(ds) - 1)[1]
        assert sorted(cat) == sorted(np.flatnonzero(t <= last))  # each row exactly once
        assert (t[wins[0]] <= ds._ref_time(0)[1]).all()  # frame 0: (-inf, t_0]
        assert len(wins[0]) > 0  # rows before frame 0 arrive in frame 0
        for k in range(1, len(ds)):
            lo, hi = ds._ref_time(k - 1)[1], ds._ref_time(k)[1]
            assert ((t[wins[k]] > lo) & (t[wins[k]] <= hi)).all()
        assert len(ds.get_vehicle_odom_data(3)) == len(wins[3])
        assert (ds.row_stamps("vehicle_odom", 3) == t[wins[3]]).all()


def test_grid_frame_transform(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with CpslDS(p, reference="grid:50", frame="base_link") as g, \
            CpslDS(p, reference="grid:50") as raw, CpslDS(p, sync=False, frame="base_link") as b:
        for k in range(len(g)):
            m = g.match("lidar", k)
            exp = b.get_lidar_point_cloud_raw(m.j) if m.j >= 0 else None
            got = g.get_lidar_point_cloud_raw(k)
            if exp is None:
                assert got.shape[0] == 0
            else:
                assert np.allclose(got, exp) and got.shape == raw.get_lidar_point_cloud_raw(k).shape


def test_intersection_vs_union_fallback(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        with CpslDS(p, reference="grid:100") as ds:  # overlapping streams: no warning
            t0, t1 = span(ds)
            assert ds._ref_time(0)[1] == t0 and ds._ref_time(ds.num_frames - 1)[1] <= t1
    # shift one stream so that nothing overlaps: union + warning
    with h5py.File(p, "r+") as f:
        for k in ("stamp_recv_ns", "stamp_header_ns"):
            if k in f["cam"]:
                f["cam"][k][...] = f["cam"][k][...] + 10_000_000_000
    with pytest.warns(UserWarning, match="do not overlap"):
        ds = CpslDS(p, reference="grid:10")
    with ds:
        lo = min(int(t.min()) for t in ds._t.values() if len(t))
        hi = max(int(t.max()) for t in ds._t.values() if len(t))
        assert ds._ref_time(0)[1] == lo and ds._ref_time(-1)[1] <= hi < ds._ref_time(-1)[1] + 10**8
        assert ds.num_frames == (hi - lo) // 10**8 + 1


def test_camera_info_excluded_from_span(tmp_path):
    p = build_getters(tmp_path / "g.h5")
    with CpslDS(p, reference="grid:10") as a:
        assert "camera_info" in a.roles
        n, t0 = a.num_frames, a._ref_time(0)[1]
    with h5py.File(p, "r+") as f:  # a static sample stamped far away must not move the span
        for k in ("stamp_recv_ns", "stamp_header_ns"):
            f["cam_info"][k][...] = f["cam_info"][k][...] + 50_000_000_000
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        with CpslDS(p, reference="grid:10") as b:
            assert (b.num_frames, b._ref_time(0)[1]) == (n, t0)


def test_default_grid_when_no_sensor_reference(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with pytest.warns(UserWarning, match="10 Hz time grid.*reference="):
        ds = CpslDS(p, radar_adc=False, lidar=False, camera=False)
    with ds:
        assert ds.reference == "grid" and ds.grid_hz == 10.0 and ds.num_frames > 0
        assert ds.get_vehicle_odom_data(0).shape[1] == 14
        assert "ref=grid" in repr(ds)
    with warnings.catch_warnings():
        warnings.simplefilter("error")  # explicit choices do not warn
        with CpslDS(p, radar_adc=False, lidar=False, camera=False,
                    reference="vehicle_odom") as ds:
            assert ds.reference == "vehicle_odom" and ds.num_frames == 20
            assert ds.get_vehicle_odom_data(3)[:, 1].tolist() == [3.0]
        with CpslDS(p, radar_adc=False, lidar=False, camera=False, reference="grid:5") as ds:
            assert ds.grid_hz == 5.0
        with CpslDS(p, radar_adc=False, lidar=False, camera=False, sync=False) as ds:
            assert ds.get_vehicle_odom_data(0).shape[1] == 14


def test_all_empty_capture_has_no_frames(tmp_path):
    p = mixed(tmp_path / "m.h5")
    with pytest.warns(UserWarning):
        ds = CpslDS(p, radar_adc=False, lidar=False, camera=False, vehicle_odom=False,
                    vehicle_vel=False)
    with ds:
        assert ds.num_frames == 0
        with pytest.raises(IndexError):
            ds._ref_time(0)


# --- regression: sensor-reference captures are untouched --------------------------
def _digest(paths):
    h = hashlib.sha256()
    for p in paths:
        for kw in ({}, {"match": "nearest"}, {"reference": "vehicle_odom"}):
            try:
                ds = CpslDS(p, **kw)
            except Exception as e:
                h.update(repr(("ERR", type(e).__name__)).encode()); continue
            with ds:
                h.update(repr((ds.reference, ds.num_frames, sorted(ds.roles.items()))).encode())
                for i in range(min(ds.num_frames, 12)):
                    for role in sorted(ds.roles):
                        m = ds.match(role, i)
                        h.update(repr((role, i, tuple(m))).encode())
                    for role in ("vehicle_odom", "imu"):
                        if role in ds.roles:
                            h.update(ds.row_stamps(role, i).tobytes())
    return h.hexdigest()



def test_sensor_reference_captures_unchanged(tmp_path):
    """Digest of reference / num_frames / roles / match / windows over several
    synthetic captures, computed on the code before the grid existed (no warning)."""
    ps = [build_all(tmp_path / "a.h5"), build_sampled(tmp_path / "s.h5")[0],
          mixed(tmp_path / "m.h5"), build_getters(tmp_path / "g.h5")]
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert _digest(ps) == "60a2c6796570e8f27acfdfa1b48e67281e9430e6c34ddd771305d0e64150e74c"
