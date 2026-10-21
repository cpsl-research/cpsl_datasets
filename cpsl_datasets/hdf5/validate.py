"""Check a capture against hdf5_format.md. ``validate(path) -> list[str]``."""
import json

import h5py
import numpy as np

from .capture import NON_STREAMS, attr_list, attr_str, short_type

IMU = "qx qy qz qw wx wy wz ax ay az".split()
ODOM = "x y z qx qy qz qw vx vy vz wx wy wz".split()
TWIST = "vx vy vz wx wy wz".split()
POSE7 = "x y z qx qy qz qw".split()
CAMINFO = (["width", "height"] + ["K%d" % i for i in range(9)]
           + ["R%d" % i for i in range(9)] + ["P%d" % i for i in range(12)]
           + ["D%d" % i for i in range(14)])

# type -> (dtype, ncols, exact columns or None)
ROW_TYPES = {
    "Imu": (np.float64, 10, IMU), "Odometry": (np.float64, 13, ODOM),
    "Twist": (np.float64, 6, TWIST), "TwistStamped": (np.float64, 6, TWIST),
    "TransformStamped": (np.float64, 7, POSE7),
    "Vector3Stamped": (np.float64, 3, ["x", "y", "z"]),
    "CameraInfo": (np.float64, 46, CAMINFO),
}
RAGGED_TYPES = ("PointCloud2", "TFMessage")
IMAGE_ENC = {  # encoding -> (dtype, channels or None)
    "mono8": (np.uint8, None), "8UC1": (np.uint8, None),
    "mono16": (np.uint16, None), "16UC1": (np.uint16, None),
    "32FC1": (np.float32, None), "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3),
    "8UC3": (np.uint8, 3), "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
}
# per-field dtype and shape kind of an ADCChirpMeta stream
META = {
    "frame_index": (np.uint32, "n"),
    "record_valid": (np.bool_, "nc"), "adc_complete": (np.bool_, "nc"),
    "has_time": (np.bool_, "nc"), "time_interpolated": (np.bool_, "nc"),
    "flags": (np.uint16, "nc"), "run_idx": (np.uint16, "nc"),
    "global_chirp_idx": (np.uint32, "nc"), "ts_ticks": (np.uint64, "nc"),
    "sat_slices": (np.uint8, "nc"), "sat_ref_lag": (np.uint8, "nc"),
    "sat_slices_aligned": (np.int16, "nc"), "t_s": (np.float64, "nc"),
    "prev_tail_sat": (np.int16, "n2"), "run_known": (np.bool_, "n"),
    "run_idx_frame": (np.uint16, "n"), "records_valid": (np.uint32, "n"),
    "records_invalid": (np.uint32, "n"), "records_other_run": (np.uint32, "n"),
}


def _check_stamps(g, where, probs):
    n = None
    for k in ("stamp_header_ns", "stamp_recv_ns"):
        if k not in g:
            probs.append("%s: missing %s" % (where, k))
            continue
        d = g[k]
        if d.dtype != np.int64 or d.ndim != 1:
            probs.append("%s/%s: expected int64 (N,), got %s %s" % (where, k, d.dtype, d.shape))
            continue
        if n is not None and d.shape[0] != n:
            probs.append("%s: stamp lengths differ (%d vs %d)" % (where, n, d.shape[0]))
        n = d.shape[0] if n is None else n
    return n


def _check_cols(a, expected, where, probs):
    got = attr_list(a.get("columns"))
    if got != expected:
        probs.append("%s: columns %s, expected %s" % (where, got, expected))


def _check_data(g, where, n, dtype, tail, probs):
    """data (n, *tail) of dtype; tail entries None are free. Returns data or None."""
    if "data" not in g:
        probs.append("%s: no data dataset but n_samples=%d" % (where, n))
        return None
    d = g["data"]
    ok = d.ndim == len(tail) + 1 and all(t is None or t == s for t, s in zip(tail, d.shape[1:]))
    if not ok or d.shape[0] != n:
        probs.append("%s/data: shape %s, expected (%d%s)" % (
            where, d.shape, n, "".join(",%s" % ("?" if t is None else t) for t in tail)))
    if d.dtype != np.dtype(dtype):
        probs.append("%s/data: dtype %s, expected %s" % (where, d.dtype, np.dtype(dtype)))
    return d


def _check_stream(g, name, probs):
    a = g.attrs
    where = name
    n = _check_stamps(g, where, probs)
    for k in ("msg_type", "n_samples"):
        if k not in a:
            probs.append("%s: missing attr %s" % (where, k))
    if "msg_type" not in a or n is None:
        return
    ns = int(a.get("n_samples", -1))
    if ns != n:
        probs.append("%s: n_samples=%d but stamps have %d" % (where, ns, n))
    t = short_type(a["msg_type"])
    ragged = "row_count" in g
    if t in RAGGED_TYPES:
        if not ragged and n:
            probs.append("%s: ragged type without row_count" % where)
    elif ragged:
        probs.append("%s: unexpected row_count on %s" % (where, t))
    if n == 0:
        if "data" in g and g["data"].shape[0] != 0:
            probs.append("%s: n_samples=0 but data has %d rows" % (where, g["data"].shape[0]))
        return
    if t == "ADCDataCube":
        d = _check_data(g, where, n, np.int16, [2, None, None, None], probs)
        if attr_list(a.get("columns")) != ["real", "imag"]:
            probs.append("%s: columns %s, expected ['real', 'imag']" % (
                where, attr_list(a.get("columns"))))
        if "axes" in a and len(attr_list(a["axes"])) != 4:
            probs.append("%s: axes should have 4 labels" % where)
        if "radar_cfg_status" in a:
            st = attr_str(a["radar_cfg_status"])
            if st == "ok":
                for k in ("radar_cfg_text", "radar_cfg_sha256", "radar_cfg_metrics"):
                    if k not in a:
                        probs.append("%s: radar_cfg_status ok but no %s" % (where, k))
                try:
                    json.loads(attr_str(a.get("radar_cfg_metrics", "{}")))
                except ValueError:
                    probs.append("%s: radar_cfg_metrics is not JSON" % where)
            elif st != "not_received" and not st.startswith("unreadable"):
                probs.append("%s: odd radar_cfg_status %r" % (where, st))
    elif t == "ADCChirpMeta":
        c = None
        for k, (dt, kind) in META.items():
            if k not in g:
                probs.append("%s: missing field %s" % (where, k))
                continue
            d = g[k]
            if d.dtype != np.dtype(dt):
                probs.append("%s/%s: dtype %s, expected %s" % (where, k, d.dtype, np.dtype(dt)))
            exp = {"n": (n,), "n2": (n, 2)}.get(kind)
            if kind == "nc":
                if d.ndim != 2 or d.shape[0] != n:
                    probs.append("%s/%s: shape %s, expected (%d,C)" % (where, k, d.shape, n))
                    continue
                c = d.shape[1] if c is None else c
                if d.shape[1] != c:
                    probs.append("%s/%s: C=%d differs from %d" % (where, k, d.shape[1], c))
            elif d.shape != exp:
                probs.append("%s/%s: shape %s, expected %s" % (where, k, d.shape, exp))
    elif t == "PointCloud2":
        _check_ragged(g, where, n, a, probs, (np.float32, np.float64), None)
    elif t == "TFMessage":
        _check_ragged(g, where, n, a, probs, (np.float64,), 7)
        _check_cols(a, POSE7, where, probs)
    elif t == "Image":
        enc = attr_str(a.get("encoding", ""))
        if enc not in IMAGE_ENC:
            probs.append("%s: unknown image encoding %r" % (where, enc))
            return
        dt, ch = IMAGE_ENC[enc]
        _check_data(g, where, n, dt, [None, None] + ([ch] if ch else []), probs)
    elif t in ROW_TYPES:
        dt, k, cols = ROW_TYPES[t]
        _check_data(g, where, n, dt, [k], probs)
        _check_cols(a, cols, where, probs)
    else:
        probs.append("%s: unknown msg_type %s" % (where, attr_str(a["msg_type"])))


def _check_ragged(g, where, n, a, probs, dtypes, k):
    rc = g.get("row_count")
    if rc is None:
        return
    if rc.dtype != np.int64 or rc.shape != (n,):
        probs.append("%s/row_count: %s %s, expected int64 (%d,)" % (where, rc.dtype, rc.shape, n))
        return
    total = int(rc[:].sum())
    if "data" not in g:
        if total:
            probs.append("%s: row_count sums to %d but there is no data" % (where, total))
        return
    d = g["data"]
    if d.shape[0] != total:
        probs.append("%s: row_count sums to %d but data has %d rows" % (where, total, d.shape[0]))
    if d.dtype not in [np.dtype(x) for x in dtypes]:
        probs.append("%s/data: dtype %s, expected one of %s" % (where, d.dtype, [np.dtype(x).name for x in dtypes]))
    cols = attr_list(a.get("columns"))
    kk = k if k is not None else len(cols)
    if d.ndim != 2 or d.shape[1] != kk or len(cols) != d.shape[-1]:
        probs.append("%s: data %s does not match columns %s" % (where, d.shape, cols))


def _check_tf_static(g, probs):
    names = ("parent", "child", "xyz", "quat_xyzw", "stamp_ns", "recv_ns")
    for k in names:
        if k not in g:
            probs.append("tf_static: missing %s" % k)
    if any(k not in g for k in names):
        return
    n = g["parent"].shape[0]
    want = {"child": (n,), "xyz": (n, 3), "quat_xyzw": (n, 4), "stamp_ns": (n,), "recv_ns": (n,)}
    for k, s in want.items():
        if g[k].shape != s:
            probs.append("tf_static/%s: shape %s, expected %s" % (k, g[k].shape, s))
    for k in ("xyz", "quat_xyzw"):
        if g[k].dtype != np.float64:
            probs.append("tf_static/%s: dtype %s, expected float64" % (k, g[k].dtype))
    for k in ("stamp_ns", "recv_ns"):
        if g[k].dtype != np.int64:
            probs.append("tf_static/%s: dtype %s, expected int64" % (k, g[k].dtype))
    if "n_transforms" in g.attrs and int(g.attrs["n_transforms"]) != n:
        probs.append("tf_static: n_transforms=%d but %d rows" % (int(g.attrs["n_transforms"]), n))


def _check_tf_pair(g, name, probs):
    n = _check_stamps(g, name, probs)
    for k in ("parent_frame", "child_frame", "n_samples"):
        if k not in g.attrs:
            probs.append("%s: missing attr %s" % (name, k))
    if n is None:
        return
    if int(g.attrs.get("n_samples", n)) != n:
        probs.append("%s: n_samples=%d but stamps have %d" % (name, int(g.attrs["n_samples"]), n))
    if n:
        _check_data(g, name, n, np.float64, [7], probs)
        _check_cols(g.attrs, POSE7, name, probs)


def validate(path):
    """Return a list of problems (empty if the file conforms). Never raises on
    a malformed file; an unreadable one is reported as a problem."""
    probs = []
    try:
        f = h5py.File(path, "r")
    except OSError as e:
        return ["cannot open: %s" % e]
    with f:
        a = f.attrs
        if attr_str(a.get("format", "")) != "cpsl_dataset_hdf5":
            probs.append("root: format is %r, expected 'cpsl_dataset_hdf5'" % attr_str(a.get("format", "")))
        ver = a.get("format_version")
        if ver is None:
            probs.append("root: missing format_version")
            ver = 0
        elif not 2 <= int(ver) <= 4:
            probs.append("root: format_version %s not in 2..4" % ver)
        if "closed_cleanly" not in a:
            probs.append("root: missing closed_cleanly")
        if "start_recv_ns" not in a:
            probs.append("root: missing start_recv_ns")
        if "config_streams" in a:
            try:
                json.loads(attr_str(a["config_streams"]))
            except ValueError:
                probs.append("root: config_streams is not JSON")
        # detect by presence; a v4 file must carry /tf_static
        if int(ver) >= 4 and "tf_static" not in f:
            probs.append("root: format_version %s but no /tf_static" % ver)
        if "tf_static" in f:
            _check_tf_static(f["tf_static"], probs)
        if "tf" in f:
            for k, g in f["tf"].items():
                _check_tf_pair(g, "tf/" + k, probs)
        for name in f:
            if name in NON_STREAMS:
                continue
            if isinstance(f[name], h5py.Group):
                _check_stream(f[name], name, probs)
            else:
                probs.append("%s: not a group" % name)
    return probs
