"""B210 streams (SarBurst / SarClockSync): validate, role binding, pairing, reference
(b210-21 Step 2). Synthetic files only; the real-file check is a Runner step."""
import warnings

import h5py
import numpy as np
import pytest
from synth_h5 import B210_NS, T0, build_b210

from cpsl_datasets import CpslDS
from cpsl_datasets.hdf5 import Capture, validate


def corrupt(src, dst, fn):
    import shutil
    shutil.copy(src, dst)
    with h5py.File(dst, "r+") as f:
        fn(f)
    return dst


@pytest.mark.parametrize("kw", [{}, {"iq": False}, {"devices": ("a", "b")},
                                {"devices": ("a", "b"), "iq": False}])
def test_validate_clean(tmp_path, kw):
    assert validate(build_b210(tmp_path / "b.h5", **kw)) == []


@pytest.mark.parametrize("name,fn,frag", [
    ("dtype", lambda f: (f["b210_bursts"].__delitem__("valid"),
                         f["b210_bursts"].create_dataset("valid", data=np.zeros(6, np.uint8))),
     "valid: dtype"),
    ("missing", lambda f: f["b210_bursts"].__delitem__("usrp_ticks"), "missing field usrp_ticks"),
    ("shape", lambda f: (f["b210_bursts"].__delitem__("sweep"),
                         f["b210_bursts"].create_dataset("sweep", data=np.zeros(5, np.uint32))),
     "sweep: shape"),
    ("iq_dtype", lambda f: (f["b210_bursts"].__delitem__("iq"),
                            f["b210_bursts"].create_dataset("iq", data=np.zeros((6, 2 * B210_NS), np.int32))),
     "iq: dtype"),
    ("iq_shape", lambda f: (f["b210_bursts"].__delitem__("iq"),
                            f["b210_bursts"].create_dataset("iq", data=np.zeros((6, 3), np.int16))),
     "iq: shape"),
    ("sync_shape", lambda f: (f["b210_clock_sync"].__delitem__("data"),
                              f["b210_clock_sync"].create_dataset("data", data=np.zeros((2, 3), np.int64))),
     "b210_clock_sync/data"),
    ("sync_cols", lambda f: f["b210_clock_sync"].attrs.__setitem__("columns", ["a", "b", "c", "d"]),
     "columns"),
])
def test_validate_catches(tmp_path, name, fn, frag):
    src = build_b210(tmp_path / "ok.h5")
    probs = validate(corrupt(src, tmp_path / (name + ".h5"), fn))
    assert any(frag in p for p in probs), probs


def test_odd_iq_layout_warns(tmp_path):
    with pytest.warns(UserWarning, match="iq_layout"):
        assert validate(build_b210(tmp_path / "b.h5", layout="planar float32")) == []


def test_bind_single_device_and_stream_read(tmp_path):
    p = build_b210(tmp_path / "b.h5")
    with CpslDS(p) as ds:
        assert ds.roles == {"b210_bursts": "b210_bursts", "b210_clock_sync": "b210_clock_sync"}
    with Capture(p) as c:
        r = c["b210_bursts"].read(1)
        assert r["iq"].shape == (2 * B210_NS,) and r["iq"].dtype == np.int16
        assert r["iq"][0] == 100 and r["iq"][1] == -100  # I, Q of burst 1 sample 0
        assert int(r["index"]) == 1 and bool(r["valid"]) and int(r["num_samples"]) == B210_NS
        assert "iq" not in c["b210_bursts"].read(1, iq=False)
        assert c["b210_clock_sync"].read(0).shape == (4,)
    with Capture(build_b210(tmp_path / "n.h5", iq=False)) as c:
        assert "iq" not in c["b210_bursts"].read(1)


def test_b210_only_picks_bursts_as_reference_without_warning(tmp_path):
    p = build_b210(tmp_path / "b.h5")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        with CpslDS(p) as ds:
            assert ds.reference == "b210_bursts" and ds.grid_hz is None
            assert ds.num_frames == 6
            m = ds.match("b210_bursts", 3)
            assert m.j == 3 and m.dt_ns == 0
            assert ds.get_b210_meta(3)["index"] == 3
            assert ds.get_b210_meta(2)["valid"] == False  # noqa: E712 (numpy bool)
            assert "iq" not in ds.get_b210_meta(0)
            # clock sync is matched on its recv stamp, previous sample
            assert ds.match("b210_clock_sync", 5).j == 1
            assert ds.match("b210_clock_sync", 0).j == -1  # none yet
            st = ds.b210_clock_sync()
            assert st.shape == (2, 4) and st.dtype == np.int64
            # a burst-centre stamp row is the header stamp: host realtime at the centre
            assert ds.row_stamps("b210_bursts", 0)[0] == T0 + 8_000_000


def test_b210_only_grid_when_asked_and_header_clock(tmp_path):
    p = build_b210(tmp_path / "b.h5")
    with CpslDS(p, reference="grid") as ds:
        assert ds.reference == "grid" and ds.num_frames > 0
        assert ds.match("b210_bursts", 2).dt_ns <= 0
    with CpslDS(p, clock="header") as ds:  # SarClockSync header stamps are -1; must still open
        assert ds.reference == "b210_bursts" and ds.match("b210_bursts", 2).t_ns == T0 + 2 * 10**8
    with pytest.warns(UserWarning, match="time grid"):
        with CpslDS(p, b210_bursts=False) as ds:
            assert ds.reference == "grid"


def test_odom_wins_over_bursts_only_when_a_sensor_role_is_earlier(tmp_path):
    p = build_b210(tmp_path / "b.h5", with_odom=True)
    with CpslDS(p) as ds:  # odom is never a default reference; bursts frame it
        assert ds.reference == "b210_bursts" and "vehicle_odom" in ds.roles
        assert ds.match("vehicle_odom", 5).j >= 0
    with CpslDS(p, reference="vehicle_odom") as ds:
        assert ds.num_frames == 4 and ds.reference == "vehicle_odom"


def test_explicit_names_and_wrong_type(tmp_path):
    p = build_b210(tmp_path / "b.h5")
    with CpslDS(p, b210_bursts="b210_bursts", b210_clock_sync=False) as ds:
        assert "b210_clock_sync" not in ds.roles
        with pytest.raises(ValueError, match="no b210_clock_sync"):
            ds.b210_clock_sync()
    with pytest.raises(ValueError, match="needs"):
        CpslDS(p, b210_bursts="b210_clock_sync")


def test_two_devices_defer_ambiguity_and_pair_by_prefix(tmp_path):
    p = build_b210(tmp_path / "b.h5", devices=("a", "b"))
    with pytest.warns(UserWarning, match="time grid"):  # ambiguous bursts: no reference
        ds = CpslDS(p)
    with ds:
        assert ds.reference == "grid" and "b210_bursts" not in ds.roles
        with pytest.raises(ValueError, match="2 streams match.*a_bursts.*b_bursts"):
            ds.get_b210_meta(0)
        with pytest.raises(ValueError, match="2 streams match"):
            ds.b210_clock_sync()
    with CpslDS(p, b210_bursts="b_bursts") as ds:
        assert ds.roles["b210_clock_sync"] == "b_clock_sync" and ds.reference == "b210_bursts"
        assert ds.row_stamps("b210_bursts", 0)[0] == T0 + 1_000_000 + 8_000_000
    with CpslDS(p, b210_bursts="a_bursts", b210_clock_sync="b_clock_sync") as ds:
        assert ds.roles["b210_clock_sync"] == "b_clock_sync"  # explicit override wins


def test_clock_sync_pairing_falls_back_to_the_only_one(tmp_path):
    p = build_b210(tmp_path / "b.h5", devices=("a", "b"), sync_for=("b",))
    with CpslDS(p, b210_bursts="a_bursts") as ds:
        assert ds.roles["b210_clock_sync"] == "b_clock_sync"
    p = build_b210(tmp_path / "c.h5", devices=("x",))
    with CpslDS(p) as ds:  # prefixed single device: names pair with each other
        assert ds.roles == {"b210_bursts": "x_bursts", "b210_clock_sync": "x_clock_sync"}


def test_empty_b210_streams(tmp_path):
    p = build_b210(tmp_path / "e.h5", n=0)
    assert validate(p) == []
    with pytest.warns(UserWarning, match="time grid"):
        ds = CpslDS(p)
    with ds:
        assert ds.num_frames == 0
