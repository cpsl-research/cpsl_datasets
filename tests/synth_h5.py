"""Synthetic cpsl_dataset_hdf5 captures, written by hand with h5py + numpy.

Follows hdf5_format.md; no dependency on the Sensors recorder. Never commit
the produced files: tests build them in tmp_path.
"""
import json

import h5py
import numpy as np

T0 = 1_700_000_000_000_000_000
RECV_OFFSET_NS = 5_000_000  # recv is 5 ms after header, plus jitter

META_FIELDS = {  # name: (dtype, per-chirp shape kind)
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
CAMINFO_COLS = (["width", "height"] + [f"K{i}" for i in range(9)]
                + [f"R{i}" for i in range(9)] + [f"P{i}" for i in range(12)]
                + [f"D{i}" for i in range(14)])
CFG_METRICS = {"scheme": "tdm", "n_rx": 4, "num_samples": 4, "n_chirps": 3,
               "range_res_m": 0.05, "frame_rate_hz": 10.0}


def _strs(v):
    return np.array(v, dtype=h5py.string_dtype())


class Builder:
    def __init__(self, path, version, closed=True):
        self.f = h5py.File(path, "w")
        a = self.f.attrs
        a["format"] = "cpsl_dataset_hdf5"
        a["format_version"] = version
        a["closed_cleanly"] = closed
        a["start_recv_ns"] = T0
        a["end_recv_ns"] = T0 + 10**9
        a["namespace"] = "/"
        a["config_streams"] = "{}"
        self.rng = np.random.default_rng(0)

    def stream(self, name, msg_type, n, header=True, attrs=None, fields=None,
               ragged=None, rate_ns=100_000_000):
        g = self.f.create_group(name)
        g.attrs["topic"] = name
        g.attrs["resolved_topic"] = "/" + name
        g.attrs["msg_type"] = msg_type
        g.attrs["n_samples"] = n
        for k, v in (attrs or {}).items():
            g.attrs[k] = _strs(v) if isinstance(v, list) else v
        hdr = T0 + np.arange(n, dtype=np.int64) * rate_ns
        recv = hdr + RECV_OFFSET_NS + self.rng.integers(0, 1_000_000, n)
        g["stamp_header_ns"] = hdr if header else np.full(n, -1, np.int64)
        g["stamp_recv_ns"] = recv.astype(np.int64)
        for k, v in (fields or {}).items():
            g[k] = v
        if ragged is not None:
            g["row_count"] = np.asarray(ragged, np.int64)
        return g

    def rows(self, name, msg_type, n, cols, header=True, attrs=None, ragged=None):
        shape = (n if ragged is None else int(sum(ragged)), len(cols))
        data = self.rng.normal(size=shape)
        at = dict(attrs or {}, columns=list(cols))
        if header:
            at.setdefault("frame_id", "base_link")
        return self.stream(name, msg_type, n, header, at, {"data": data}, ragged)

    def close(self):
        self.f.close()


def build(path, variant="v4"):
    """variant: v2 | v3 | v4 | unclosed (v4 with closed_cleanly False)."""
    ver = {"v2": 2, "v3": 3, "v4": 4, "unclosed": 4}[variant]
    b = Builder(path, ver, closed=variant != "unclosed")
    rng, n = b.rng, 3
    # --- in every variant (v2 content)
    iq = rng.integers(-100, 100, size=(n, 2, 2, 4, 3)).astype(np.int16)
    cube = b.stream("radar_adc", "raw_radar_msgs/msg/ADCDataCube", n,
                    attrs={"columns": ["real", "imag"], "frame_id": "radar",
                           "axes": ["re_im", "rx", "sample", "chirp"]},
                    fields={"data": iq})
    meta = {}
    for k, (dt, kind) in META_FIELDS.items():
        shape = {"n": (n,), "nc": (n, 3), "n2": (n, 2)}[kind]
        meta[k] = (rng.integers(0, 2, shape)).astype(dt)
    b.stream("adc_meta", "raw_radar_msgs/msg/ADCChirpMeta", n, fields=meta)
    rc = [4, 0, 2]  # includes an empty cloud
    b.rows("radar_pc", "sensor_msgs/msg/PointCloud2", n,
           ["x", "y", "z", "vel"], attrs={"frame_id": "radar", "skip_nans": True},
           ragged=rc)
    for name, enc, dt, shape in (
            ("cam_rgb", "rgb8", np.uint8, (n, 5, 6, 3)),
            ("cam_mono", "mono8", np.uint8, (n, 5, 6)),
            ("depth16", "mono16", np.uint16, (n, 5, 6)),
            ("depth32", "32FC1", np.float32, (n, 5, 6))):
        b.stream(name, "sensor_msgs/msg/Image", n,
                 attrs={"encoding": enc, "frame_id": "camera"},
                 fields={"data": (rng.random(shape) * 200).astype(dt)})
    # empty stream like the real captures' radar_adc: no data dataset
    b.stream("radar_adc_empty", "raw_radar_msgs/msg/ADCDataCube", 0,
             attrs={"frame_id": "radar"})
    if ver >= 3:
        cube.attrs["radar_cfg_topic"] = "radar_config_path"
        cube.attrs["radar_cfg_status"] = "ok"
        cube.attrs["radar_cfg_path"] = "/cfg/x.cfg"
        cube.attrs["radar_cfg_text"] = "sensorStart\n"
        cube.attrs["radar_cfg_sha256"] = "0" * 64
        cube.attrs["radar_cfg_metrics"] = json.dumps(CFG_METRICS)
        cube.attrs["radar_cfg_changed"] = False
        imu_cols = "qx qy qz qw wx wy wz ax ay az".split()
        b.rows("imu", "sensor_msgs/msg/Imu", n, imu_cols)
        odom_cols = "x y z qx qy qz qw vx vy vz wx wy wz".split()
        b.rows("odom", "nav_msgs/msg/Odometry", n, odom_cols,
               attrs={"frame_id": "odom", "child_frame_id": "base_link"})
        b.rows("cmd_vel", "geometry_msgs/msg/Twist", n,
               "vx vy vz wx wy wz".split(), header=False)
    if ver >= 4:
        b.rows("twist_st", "geometry_msgs/msg/TwistStamped", n,
               "vx vy vz wx wy wz".split())
        b.rows("tstamped", "geometry_msgs/msg/TransformStamped", n,
               "x y z qx qy qz qw".split(), attrs={"child_frame_id": "tool"})
        b.rows("vec3", "geometry_msgs/msg/Vector3Stamped", n, ["x", "y", "z"])
        ci = rng.normal(size=(n, 46))
        ci[:, 37:] = np.nan  # D0..D4 real, rest NaN-padded
        b.stream("cam_info", "sensor_msgs/msg/CameraInfo", n,
                 attrs={"columns": CAMINFO_COLS, "frame_id": "camera",
                        "distortion_model": "plumb_bob"}, fields={"data": ci})
        b.rows("tfmsg", "tf2_msgs/msg/TFMessage", 2, "x y z qx qy qz qw".split(),
               header=False, ragged=[2, 3])
        b.f.attrs["tf_track"] = _strs(["odom->base_link"])
        b.f.attrs["tf_topic"] = "/tf"
        b.f.attrs["tf_static_topic"] = "/tf_static"
        b.f.attrs["robot_description"] = "<robot name='r'/>"
        sg = b.f.create_group("tf_static")
        rows = [("base_link", "radar", [0.1, 0, 0.2], [0, 0, 0.7071068, 0.7071068]),
                ("base_link", "camera", [0.0, 0.1, 0.3], [0, 0, 0, 1])]
        sg["parent"] = _strs([r[0] for r in rows])
        sg["child"] = _strs([r[1] for r in rows])
        sg["xyz"] = np.array([r[2] for r in rows], np.float64)
        sg["quat_xyzw"] = np.array([r[3] for r in rows], np.float64)
        sg["stamp_ns"] = np.array([T0, T0], np.int64)
        sg["recv_ns"] = np.array([T0 + 1, T0 + 1], np.int64)
        sg.attrs["n_transforms"] = 2
        tf = b.f.create_group("tf")
        m = 5
        hdr = T0 + np.arange(m, dtype=np.int64) * 100_000_000
        tg = tf.create_group("odom__base_link")
        d = np.zeros((m, 7))
        d[:, 0] = np.arange(m) * 0.5  # moves along x
        d[:, 6] = 1.0
        tg["data"] = d
        tg["stamp_header_ns"] = hdr
        tg["stamp_recv_ns"] = hdr + RECV_OFFSET_NS
        for k, v in (("columns", _strs("x y z qx qy qz qw".split())),
                     ("parent_frame", "odom"), ("child_frame", "base_link"),
                     ("topic", "tf"), ("resolved_topic", "/tf"),
                     ("msg_type", "geometry_msgs/msg/TransformStamped"),
                     ("n_samples", m)):
            tg.attrs[k] = v
    b.close()
    return path


def build_human_movement(path):
    """human_movement-shaped v4 capture: radar cube + radar pc, a lidar, an rgb
    camera, Leap hand clouds (21 points per sample, left hand absent in sample 1
    = zeros), Leap mono8 images, and a Vicon TransformStamped."""
    b = Builder(path, 4)
    rng, n = b.rng, 3
    b.stream("radar_adc", "raw_radar_msgs/msg/ADCDataCube", n,
             attrs={"columns": ["real", "imag"], "frame_id": "radar"},
             fields={"data": rng.integers(-9, 9, (n, 2, 2, 4, 3)).astype(np.int16)})
    b.rows("radar_pc", "sensor_msgs/msg/PointCloud2", n, ["x", "y", "z", "vel"],
           attrs={"frame_id": "radar"}, ragged=[2, 3, 1])
    b.rows("lidar", "sensor_msgs/msg/PointCloud2", n, ["x", "y", "z", "intensity"],
           attrs={"frame_id": "lidar"}, ragged=[5, 4, 6])
    b.stream("camera", "sensor_msgs/msg/Image", n,
             attrs={"encoding": "rgb8", "frame_id": "camera"},
             fields={"data": rng.integers(0, 200, (n, 5, 6, 3)).astype(np.uint8)})
    for name in ("hand_left", "hand_right"):
        g = b.rows(name, "sensor_msgs/msg/PointCloud2", n, ["x", "y", "z"],
                   attrs={"frame_id": "cpsl_human_movement/leap_link"}, ragged=[21] * n)
        if name == "hand_left":
            d = g["data"][:]
            d[21:42] = 0.0
            g["data"][...] = d
    for name in ("leap_left", "leap_right"):
        b.stream(name, "sensor_msgs/msg/Image", n,
                 attrs={"encoding": "mono8", "frame_id": "cpsl_human_movement/leap_camera_frame"},
                 fields={"data": rng.integers(0, 255, (n, 4, 6)).astype(np.uint8)})
    b.rows("vicon_box", "geometry_msgs/msg/TransformStamped", n,
           "x y z qx qy qz qw".split(), attrs={"child_frame_id": "box"})
    b.close()
    return path


def build_multi_radar(path):
    """IcaRAus-style: three radar point clouds (all with vel) and a lidar."""
    b = Builder(path, 4)
    for k in range(3):
        b.rows("radar_pc_%d" % k, "sensor_msgs/msg/PointCloud2", 3,
               ["x", "y", "z", "vel"], attrs={"frame_id": "radar%d" % k}, ragged=[2, 1, 3])
    b.rows("lidar", "sensor_msgs/msg/PointCloud2", 3, ["x", "y", "z", "intensity"],
           attrs={"frame_id": "lidar"}, ragged=[4, 4, 4])
    b.close()
    return path


def build_getters(path):
    """Capture for the secondary getters: radar cube (reference, 4 @100 ms), an
    Ouster-style organized lidar (extra columns, a NaN and a zero point), two
    Vicon segments @ 20 ms with known quaternions, IMU / Twist / Odometry @ 20 ms,
    Leap hands + images, a depth image, CameraInfo, a TFMessage."""
    b = Builder(path, 4)
    rng, n = b.rng, 4
    b.stream("radar_adc", "raw_radar_msgs/msg/ADCDataCube", n,
             attrs={"columns": ["real", "imag"], "frame_id": "radar"},
             fields={"data": rng.integers(-9, 9, (n, 2, 2, 4, 3)).astype(np.int16)})
    ocols = ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"]
    pts = rng.normal(size=(3 * n, len(ocols))).astype(np.float32) + 1.0
    pts[1, :3] = np.nan          # frame 0, point 1: NaN
    pts[2, :3] = 0.0             # frame 0, point 2: all-zero
    b.stream("lidar", "sensor_msgs/msg/PointCloud2", n,
             attrs={"columns": ocols, "frame_id": "os_sensor"},
             fields={"data": pts}, ragged=[3] * n)
    m, rate = 20, 20_000_000
    for name, off in (("vicon_a", 0.0), ("vicon_b", 10.0)):
        d = np.zeros((m, 7))
        d[:, :3] = off + np.arange(m)[:, None] * [1.0, 2.0, 3.0]
        d[:, 3:] = QUATS[np.arange(m) % len(QUATS)]
        b.stream(name, "geometry_msgs/msg/TransformStamped", m, rate_ns=rate,
                 attrs={"columns": "x y z qx qy qz qw".split(), "frame_id": "vicon_world",
                        "child_frame_id": "seg_" + name[-1]}, fields={"data": d})
    imu = np.zeros((m, 10))
    imu[:, :4] = [0, 0, np.sin(0.3), np.cos(0.3)]  # yaw 0.6
    imu[:, 4:] = np.arange(m)[:, None] + np.arange(6) * 0.1
    b.stream("imu", "sensor_msgs/msg/Imu", m, rate_ns=rate,
             attrs={"columns": "qx qy qz qw wx wy wz ax ay az".split(), "frame_id": "imu"},
             fields={"data": imu})
    vel = np.arange(m * 6, dtype=float).reshape(m, 6)
    b.stream("vehicle_vel", "geometry_msgs/msg/Twist", m, rate_ns=rate, header=False,
             attrs={"columns": "vx vy vz wx wy wz".split()}, fields={"data": vel})
    b.stream("cmd_vel", "geometry_msgs/msg/Twist", m, rate_ns=rate, header=False,
             attrs={"columns": "vx vy vz wx wy wz".split()}, fields={"data": vel * 0})
    b.rows("vec3", "geometry_msgs/msg/Vector3Stamped", m, ["x", "y", "z"])
    for name in ("hand_left", "hand_right"):
        g = b.rows(name, "sensor_msgs/msg/PointCloud2", n, ["x", "y", "z"],
                   attrs={"frame_id": "cpsl_human_movement/leap_link"}, ragged=[21] * n)
        if name == "hand_left":
            d = g["data"][:]
            d[21:42] = 0.0  # absent in sample 1
            g["data"][...] = d
    for name in ("leap_left", "leap_right"):
        b.stream(name, "sensor_msgs/msg/Image", n,
                 attrs={"encoding": "mono8", "frame_id": "cpsl_human_movement/leap_camera_frame"},
                 fields={"data": rng.integers(0, 255, (n, 4, 6)).astype(np.uint8)})
    b.stream("camera_depth", "sensor_msgs/msg/Image", n,
             attrs={"encoding": "mono16", "frame_id": "camera"},
             fields={"data": rng.integers(0, 4000, (n, 5, 6)).astype(np.uint16)})
    ci = rng.normal(size=(n, 46))
    ci[:, 37:] = np.nan
    b.stream("cam_info", "sensor_msgs/msg/CameraInfo", n,
             attrs={"columns": CAMINFO_COLS, "frame_id": "camera",
                    "distortion_model": "plumb_bob"}, fields={"data": ci})
    b.rows("tfmsg", "tf2_msgs/msg/TFMessage", 2, "x y z qx qy qz qw".split(),
           header=False, ragged=[2, 3])
    b.close()
    return path


_q = np.array([[0.1, 0.2, 0.3, 0.9], [0, 0, np.sin(0.5), np.cos(0.5)],
               [0.5, 0.5, 0.5, 0.5], [-0.2, 0.1, 0.4, 0.8]])
QUATS = _q / np.linalg.norm(_q, axis=1, keepdims=True)  # x y z w, unit
