"""CpslDS v2: v1-style frame getters over a cpsl_dataset_hdf5 capture (format v2-4)."""
import warnings

import numpy as np

from .hdf5 import Capture
from .hdf5 import tf as _tf
from .hdf5.validate import IMU as _IMU_COLS

# role -> (msg_type short name, extra predicate on the Stream, v1 column order)
_COLOR = ("rgb8", "bgr8", "rgba8", "bgra8", "8UC3")
HAND_NAMES = ("hand_left", "hand_right")
HAND_FRAME_SUFFIXES = ("leap_link", "leap_camera_frame")
N_HAND_POINTS = 21


def _is_hand(s):
    """A Leap hand-joint cloud: a PointCloud2 without ``vel`` that is named
    hand_left/hand_right, has a Leap frame_id, or has 21 points in every
    non-empty sample."""
    if s.type != "PointCloud2" or "vel" in s.columns:
        return False
    if s.name in HAND_NAMES:
        return True
    if s.frame_id and s.frame_id.endswith(HAND_FRAME_SUFFIXES):
        return True
    if s.ragged:
        rc = s._g["row_count"][:]
        rc = rc[rc > 0]
        return bool(len(rc)) and bool((rc == N_HAND_POINTS).all())
    return False


ROLES = {
    "radar_adc": ("ADCDataCube", None, None),
    "radar_pc": ("PointCloud2", lambda s: "vel" in s.columns, ("x", "y", "z", "vel")),
    "lidar": ("PointCloud2", lambda s: "vel" not in s.columns and not _is_hand(s),
              ("x", "y", "z", "intensity")),
    "hand_left": ("PointCloud2", lambda s: s.name == "hand_left" and _is_hand(s), None),
    "hand_right": ("PointCloud2", lambda s: s.name == "hand_right" and _is_hand(s), None),
    "hand": ("PointCloud2", lambda s: s.name not in HAND_NAMES and _is_hand(s), None),
    "camera": ("Image", lambda s: _enc(s) in _COLOR, None),
    "vehicle_odom": ("Odometry", None, None),
    # secondary roles: an ambiguous auto-detect is reported when a getter needs the
    # role, not at construction (real captures often hold several of these).
    "vehicle_vel": (("Twist", "TwistStamped", "Odometry"), None, None),
    "imu": ("Imu", None, None),
    "camera_depth": ("Image", lambda s: s.name == "camera_depth", None),
    "leap_left": ("Image", lambda s: s.name == "leap_left", None),
    "leap_right": ("Image", lambda s: s.name == "leap_right", None),
    "camera_info": ("CameraInfo", None, None),
    "chirp_meta": ("ADCChirpMeta", None, None),
    "vector3": ("Vector3Stamped", None, None),
    "tf_message": ("TFMessage", None, None),
}
SECONDARY = ("vehicle_vel", "imu", "camera_depth", "leap_left", "leap_right",
             "camera_info", "chirp_meta", "vector3", "tf_message")
# msg_type -> getters that serve it (tests/test_cpsl_ds.py checks this covers the
# decoder table in hdf5/validate.py and that every named getter exists)
TYPE_GETTERS = {
    "ADCDataCube": ("get_radar_adc_data",),
    "ADCChirpMeta": ("get_chirp_meta",),
    "PointCloud2": ("get_radar_point_cloud", "get_lidar_point_cloud_raw",
                    "get_point_cloud_columns", "get_hand_tracking_data"),
    "Image": ("get_camera_frame", "get_camera_depth_frame",
              "get_leap_motion_image_frame_left", "get_leap_motion_image_frame_right"),
    "Imu": ("get_imu_full_data", "get_imu_orientation_rad"),
    "Odometry": ("get_vehicle_odom_data", "get_vehicle_vel_data"),
    "Twist": ("get_vehicle_vel_data",),
    "TwistStamped": ("get_vehicle_vel_data",),
    "TransformStamped": ("get_vicon_data",),
    "Vector3Stamped": ("get_vector3",),
    "CameraInfo": ("get_camera_info",),
    "TFMessage": ("get_tf_message",),
}
REFERENCE_ORDER = ("radar_adc", "radar_pc", "lidar", "camera")
CLOUD_ROLES = ("radar_pc", "lidar")
VEL_COLS = {"Twist": (0, 5), "TwistStamped": (0, 5), "Odometry": (7, 12)}  # vx, wz
ODOM_COLS = 14  # t + x y z qx qy qz qw vx vy vz wx wy wz


def _enc(stream):
    v = stream.attrs.get("encoding", "")
    return v.decode() if isinstance(v, bytes) else str(v)


class CpslDS:
    """Frame-indexed reader for one HDF5 capture, with the v1 getter names.

    ``CpslDS(path, radar_adc=None, radar_pc=None, lidar=None, camera=None,
    vehicle_odom=None, hand_left=None, hand_right=None, hand=None,
    reference=None, sync=True, clock="recv", frame=None)``

    **Stream roles.** Each role kwarg takes a stream (group) name. ``None``
    auto-detects: the one stream whose ``msg_type`` matches (radar_adc:
    ADCDataCube; radar_pc: PointCloud2 with a ``vel`` column; lidar:
    PointCloud2 without one that is not a hand cloud; hand_left / hand_right:
    the hand cloud named ``hand_left`` / ``hand_right``; hand: a single other
    hand cloud (v1-style one ``(21, 3)`` stream); camera: Image with a colour encoding (rgb8, bgr8,
    rgba8, bgra8, 8UC3); vehicle_odom: Odometry). A *hand cloud* is a
    PointCloud2 without ``vel`` that is named hand_left/hand_right, has a
    frame_id ending ``leap_link`` / ``leap_camera_frame``, or has 21 points in
    every non-empty sample; it never binds as lidar. Two matches raise
    ValueError, empty streams included (e.g. several radars or lidars: name
    the one you want, ``radar_pc="radar_pc_1"``, or switch the role off). No
    match leaves the role absent. ``False`` or ``""`` switches a role off.
    A role bound to an empty stream (a stream that never delivered) is
    present with zero samples. ``roles`` maps role -> stream name.

    **Index (sync=True, the default).** ``idx`` is a sample of the
    *reference* stream: ``reference=`` (a role name), else the first of
    radar_adc, radar_pc, lidar, camera that is bound and non-empty.
    ``num_frames`` is its length. Every other getter returns that stream's
    newest sample stamped at or before the reference sample (reading_datasets
    section 3 ``last``; ties go to the later row). When a stream has no
    sample yet, the getter returns an empty array (shapes below), as v1 did
    for an unreadable file. With no reference (e.g. a capture holding only
    odometry) ``num_frames`` is 0 and synced getters raise; pass
    ``reference="vehicle_odom"`` or ``sync=False``.
    **sync=False** makes ``idx`` the role's own raw sample index (out of
    range raises IndexError); ``num_frames`` is still the reference length.

    **Clock.** ``clock="recv"`` (default) aligns on ``stamp_recv_ns``, the
    recording host's clock, shared by every stream. ``clock="header"`` uses
    ``stamp_header_ns``, which is only meaningful when the drivers share a
    clock; a bound stream without header stamps (-1) then raises. The default
    is a default, not a guarantee of best alignment: on real captures the
    radar header trails recv by ~104-126 ms with ~5 ms jitter while odometry
    is within ~0.6 ms, so recv and header pick different samples. Check
    ``python -m cpsl_datasets.inspect`` on your capture. The choice applies
    to sync lookups, ``get_stamps``, odom time and ``get_transform``.

    **Frame.** Point clouds come back in their sensor frame as stored
    (``frame_id`` attr). ``frame="<base>"`` maps x, y, z of radar_pc and
    lidar into that frame with the static chain from ``/tf_static`` (format
    v4 only; v2/v3 or no static path raises ValueError at construction);
    other columns are untouched. Note: v1's "sensor frame" clouds were
    already in the preset's base frame, so v1 output corresponds to
    ``frame=<that base>``. ADC cubes and images are never transformed.

    **Getters and return shapes.**

    - ``get_radar_adc_data(idx)``: complex64 ``(rx, sample, chirp)``;
      empty ``(0, 0, 0)``.
    - ``get_radar_point_cloud(idx)``: ``(n, 4)`` x y z vel when those columns
      exist, else all stored columns; empty ``(0, K)``.
    - ``get_lidar_point_cloud_raw(idx)``: ``(n, 4)`` x y z intensity when
      present, else all stored columns. ``get_lidar_point_cloud(idx)``:
      ``(n, 2)`` x y of points with -0.2 < z < 0.1, v1's ground/ceiling cut,
      applied in the returned frame.
    - ``get_camera_frame(idx)``: the image as stored, ``(H, W[, C])`` in
      ``camera_encoding`` (v1 returned RGB from a PNG); empty ``(0, 0, 0)``.
    - ``get_vehicle_odom_data(idx)``: ``(m, 14)``: time in seconds (chosen
      clock) then the 13 stored columns ``x y z qx qy qz qw vx vy vz wx wy
      wz``. Quaternions are x-first as stored; v1 was w-first. Synced: the
      odom rows in the window ``(t_ref[idx-1], t_ref[idx]]``, ``(-inf,
      t_ref[0]]`` for idx 0 (reading_datasets section 3 ``win``), so every
      row up to the last reference sample lands in exactly one frame.
      Unsynced: row ``idx`` as ``(1, 14)``.
    - ``get_stamps(role)``: int64 ns of the role's whole stream, chosen clock.
    - ``get_radar_cfg()``: the cube's parsed ``radar_cfg_*`` attrs (format
      v3+; ``metrics`` decoded), or None on every capture where no cfg was
      recorded (v2, converted v1 data, status other than ``ok``). The reader
      returns the raw ADC cube and this cfg as stored; it does no radar signal
      processing (range-Doppler etc. lives in radar_gui / CPSL_TI_Radar).
    - Row streams, windowed like odometry (``(m, ...)`` rows with
      ``t`` in seconds first): ``get_vehicle_vel_data`` ``t vx wz``,
      ``get_imu_full_data`` ``t wx wy wz ax ay az``, ``get_vector3``
      ``t x y z``, ``get_vicon_data`` ``x y z qx qy qz qw`` (no time column;
      ``wxyz=True`` for v1's order; a dict per stream when several
      ``vicon_<segment>``; frames in ``vicon_frames``). These are secondary
      roles (``vehicle_vel``, ``imu``, ``vector3``, ``vicon``, ...):
      an ambiguous auto-detect is reported when the getter is called, not at
      construction. ``vehicle_vel`` falls back to the odometry stream.
    - ``get_hand_tracking_data(idx, hand=None)`` ``(21, 3)`` and
      ``hand_present``: raw Leap-sensor metres, never transformed (see its
      docstring); ``get_leap_motion_image_frame_left/right``,
      ``get_camera_depth_frame``: images as stored.
    - ``get_point_cloud_columns(idx, role="lidar", drop_invalid=False)``: every
      stored column as ``{name: array}`` (Ouster ``t``, ``ring``, ...).
    - ``get_imu_orientation_rad``, ``get_camera_info``, ``get_chirp_meta``,
      ``get_tf_message``; deprecated ``get_radar_data``.
    - v1 getters without a v2 counterpart: none. v1 ``import_*`` /
      ``print_available_folders`` / ``load_new_dataset`` are replaced by
      construction; v1 per-frame ``.npy`` layouts by the shapes above.
    - ``get_transform(target, source, idx=None, t_ns=None)``: 4x4
      ``T_target_source`` over ``/tf_static`` and the tracked ``/tf`` pairs
      (v4). A tracked edge is interpolated at ``t_ns``, or at the reference
      stamp of ``idx``, on the chosen clock; it raises if neither is given.

    Use as a context manager or call ``close()``.
    """

    def __init__(self, dataset_path, radar_adc=None, radar_pc=None, lidar=None,
                 camera=None, vehicle_odom=None, hand_left=None, hand_right=None,
                 hand=None, reference=None, sync=True,
                 clock="recv", frame=None, vehicle_vel=None, imu=None,
                 camera_depth=None, leap_left=None, leap_right=None,
                 camera_info=None, chirp_meta=None, vector3=None, tf_message=None,
                 vicon=None):
        if clock not in ("recv", "header"):
            raise ValueError("clock must be 'recv' or 'header', got %r" % (clock,))
        self.dataset_path = str(dataset_path)
        self.sync = bool(sync)
        self.clock = clock
        self.frame = frame
        self.capture = Capture(self.dataset_path)
        try:
            self._setup(dict(radar_adc=radar_adc, radar_pc=radar_pc, lidar=lidar,
                             camera=camera, vehicle_odom=vehicle_odom,
                             hand_left=hand_left, hand_right=hand_right,
                             hand=hand, vehicle_vel=vehicle_vel, imu=imu,
                             camera_depth=camera_depth, leap_left=leap_left,
                             leap_right=leap_right, camera_info=camera_info,
                             chirp_meta=chirp_meta, vector3=vector3,
                             tf_message=tf_message), reference, vicon)
        except Exception:
            self.capture.close()
            raise

    # --- setup ----------------------------------------------------------
    def _setup(self, requested, reference, vicon=None):
        all_streams = self.capture.streams()
        self.streams = {}
        self._deferred = {}
        for role, name in requested.items():
            try:
                s = self._bind(role, name, all_streams)
            except ValueError as e:
                if role in SECONDARY and name is None:
                    self._deferred[role] = str(e)
                    continue
                raise
            if s is not None:
                self.streams[role] = s
        if (requested["vehicle_vel"] is None and "vehicle_vel" not in self.streams
                and "vehicle_vel" not in self._deferred and "vehicle_odom" in self.streams):
            self.streams["vehicle_vel"] = self.streams["vehicle_odom"]  # Odometry fallback
        self.vicon_names = []
        if vicon is not False and vicon != "":
            if vicon is None:
                names = [n for n, s in all_streams.items() if s.type == "TransformStamped"]
            else:
                names = [vicon] if isinstance(vicon, str) else list(vicon)
                for n in names:
                    if n not in all_streams:
                        raise KeyError("vicon=%r: no such stream in %s (streams: %s)"
                                       % (n, self.dataset_path, sorted(all_streams)))
            for n in sorted(names):
                self.streams[n] = all_streams[n]
                self.vicon_names.append(n)
        self.roles = {r: s.name for r, s in self.streams.items()}
        self._t, self._order, self._sorted = {}, {}, {}
        for role, s in self.streams.items():
            t = s.stamps(self.clock)
            if self.clock == "header" and len(t) and (t < 0).any():
                raise ValueError("%s (%s) has no header stamps (-1); use clock='recv'"
                                 % (role, s.name))
            order = np.argsort(t, kind="stable")
            if len(t) > 1 and (np.diff(t) < 0).any():
                warnings.warn("%s: %s stamps are not monotonic; lookups use sorted order"
                              % (s.name, self.clock), stacklevel=3)
            self._t[role], self._order[role], self._sorted[role] = t, order, t[order]
        if reference is None:
            reference = next((r for r in REFERENCE_ORDER
                              if r in self.streams and len(self.streams[r])), None)
        elif reference not in self.streams:
            raise ValueError("reference=%r is not a bound role (bound: %s)"
                             % (reference, sorted(self.streams)))
        self.reference = reference
        self.num_frames = len(self.streams[reference]) if reference else 0
        self._T = {}
        if self.frame is not None:
            self._setup_frame()

    def _bind(self, role, name, all_streams):
        if name is False or name == "":
            return None
        mtype, pred, _ = ROLES[role]
        types = (mtype,) if isinstance(mtype, str) else mtype
        if name is not None:
            if name not in all_streams:
                raise KeyError("%s=%r: no such stream in %s (streams: %s)"
                               % (role, name, self.dataset_path, sorted(all_streams)))
            if all_streams[name].type not in types:
                raise ValueError("%s=%r is a %s stream; this role needs %s"
                                 % (role, name, all_streams[name].type, "/".join(types)))
            return all_streams[name]
        hits = [s for s in all_streams.values()
                if s.type in types and (pred is None or pred(s))]
        if role == "vehicle_vel":  # Twist/TwistStamped; Odometry only as the fallback
            hits = [s for s in hits if s.type != "Odometry"]
            named = [s for s in hits if s.name == "vehicle_vel"]
            hits = named or hits
        if len(hits) > 1:
            raise ValueError("%s: %d streams match (%s); pass %s=<name> or %s=False"
                             % (role, len(hits), ", ".join(sorted(s.name for s in hits)),
                                role, role))
        return hits[0] if hits else None

    def _setup_frame(self):
        if not self.capture.tf_static():
            raise ValueError(
                "frame=%r needs /tf_static, which this file (format v%d) does not have; "
                "use frame=None for clouds in their sensor frame"
                % (self.frame, self.capture.version))
        for role in CLOUD_ROLES:
            s = self.streams.get(role)
            if s is None or not len(s):
                continue
            if not s.frame_id:
                raise ValueError("%s (%s) has no frame_id; cannot map it to %r"
                                 % (role, s.name, self.frame))
            try:
                self._T[role] = self.capture.static_chain(s.frame_id, self.frame)
            except ValueError as e:
                raise ValueError("frame=%r for %s (%s): %s (frame=None keeps the sensor frame)"
                                 % (self.frame, role, s.name, e)) from None

    # --- lifecycle --------------------------------------------------------
    def close(self):
        self.capture.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __len__(self):
        return self.num_frames

    def __repr__(self):
        return "<CpslDS %s frames=%d ref=%s roles=%s>" % (
            self.dataset_path, self.num_frames, self.reference, self.roles)

    # --- indexing -----------------------------------------------------------
    def _stream(self, role):
        if role in self._deferred:
            raise ValueError(self._deferred[role])
        if role not in self.streams:
            raise ValueError("no %s stream in %s (roles: %s)"
                             % (role, self.dataset_path, self.roles))
        return self.streams[role]

    def _ref_time(self, idx):
        if self.reference is None:
            raise RuntimeError("no reference stream with samples in %s; pass reference=<role> "
                               "or sync=False" % self.dataset_path)
        n = self.num_frames
        i = int(idx)
        if i < 0:
            i += n
        if not 0 <= i < n:
            raise IndexError("frame %d out of range (%d)" % (int(idx), n))
        return i, int(self._t[self.reference][i])

    def sample_index(self, role, idx):
        """Raw sample index of ``role`` served for ``idx``; -1 = none yet."""
        s = self._stream(role)
        if not self.sync:
            return s._index(idx)
        i, t = self._ref_time(idx)
        if role == self.reference:
            return i
        k = int(np.searchsorted(self._sorted[role], t, side="right")) - 1
        return int(self._order[role][k]) if k >= 0 else -1

    # --- getters ------------------------------------------------------------
    def get_stamps(self, role):
        self._stream(role)
        return self._t[role].copy()

    def get_radar_cfg(self):
        s = self.streams.get("radar_adc")
        cfg = self.capture.radar_cfg(s.name if s is not None else None)
        if not cfg or cfg.get("status") != "ok":
            return None
        return cfg

    def get_radar_adc_data(self, idx):
        j = self.sample_index("radar_adc", idx)
        if j < 0:
            return np.empty((0, 0, 0), np.complex64)
        return self.streams["radar_adc"].read(j)

    def _cloud(self, role, idx, full=False):
        """(points, column names) of a cloud role, frame applied; ``full`` keeps
        every stored column."""
        s = self._stream(role)
        j = self.sample_index(role, idx)
        cols = list(s.columns)
        want = ROLES[role][2]
        sel = None if full else [cols.index(c) for c in want] if all(c in cols for c in want) else None
        out_cols = list(want) if sel is not None else cols
        if j < 0:
            return np.empty((0, len(out_cols)), np.float32), out_cols
        pts = np.asarray(s.read(j))
        if role in self._T and len(pts):
            xyz = [cols.index(c) for c in ("x", "y", "z")]
            T = self._T[role]
            p = pts[:, xyz].astype(np.float64)
            pts = pts.copy()
            pts[:, xyz] = (p @ T[:3, :3].T + T[:3, 3]).astype(pts.dtype)
        return (pts[:, sel] if sel is not None else pts), out_cols

    def get_radar_point_cloud(self, idx):
        return self._cloud("radar_pc", idx)[0]

    def get_lidar_point_cloud_raw(self, idx):
        return self._cloud("lidar", idx)[0]

    def get_lidar_point_cloud(self, idx):
        pts, cols = self._cloud("lidar", idx)
        if not all(c in cols for c in ("x", "y", "z")):
            raise ValueError("lidar stream has no x y z columns: %s" % cols)
        ix, iy, iz = (cols.index(c) for c in ("x", "y", "z"))
        keep = (pts[:, iz] > -0.2) & (pts[:, iz] < 0.1)
        return pts[keep][:, [ix, iy]]

    @property
    def camera_encoding(self):
        s = self.streams.get("camera")
        return _enc(s) if s is not None else None

    def get_camera_frame(self, idx):
        j = self.sample_index("camera", idx)
        if j < 0:
            return np.empty((0, 0, 0), np.uint8)
        return self.streams["camera"].read(j)

    # --- row streams (time series) ------------------------------------------
    def _window(self, role, idx):
        """Row indices of ``role`` for frame ``idx``: synced = the rows in
        ``(t_ref[idx-1], t_ref[idx]]`` (``(-inf, t_ref[0]]`` for idx 0), time
        ordered; the reference stream itself gives row ``idx``. Unsynced: row ``idx``."""
        s = self._stream(role)
        t = self._t[role]
        if not self.sync:
            return np.array([s._index(idx)])
        i, t1 = self._ref_time(idx)
        if self.roles[self.reference] == self.roles[role]:
            return np.array([i])
        lo = int(self._t[self.reference][i - 1]) if i > 0 else None
        m = t <= t1 if lo is None else (t > lo) & (t <= t1)
        rows = np.flatnonzero(m)
        return rows[np.argsort(t[rows], kind="stable")]

    def _rows(self, role, idx, cols=None, with_time=True):
        """``(m, [1+]k)`` float rows of a row-type stream's window; ``cols`` are
        column positions (None = all). Time is seconds on the chosen clock."""
        rows = self._window(role, idx)
        s = self.streams[role]
        k = s._g["data"].shape[1] if "data" in s._g else 0
        cols = list(range(k)) if cols is None else list(cols)
        if not len(rows):
            return np.empty((0, len(cols) + int(with_time)))
        srt = np.sort(rows)
        data = s._g["data"][srt][np.searchsorted(srt, rows)][:, cols]
        if not with_time:
            return data
        return np.c_[self._t[role][rows] * 1e-9, data]

    def get_vehicle_odom_data(self, idx=0):
        return self._rows("vehicle_odom", idx)

    def get_vehicle_vel_data(self, idx=0):
        """``(m, 3)`` ``t vx wz`` (v1 layout): from a Twist / TwistStamped stream,
        else from the odometry stream's twist. Window as ``get_vehicle_odom_data``.
        Several Twist streams: pass ``vehicle_vel="<name>"`` (a stream named
        ``vehicle_vel`` is chosen automatically)."""
        self._stream("vehicle_vel")
        return self._rows("vehicle_vel", idx, VEL_COLS[self.streams["vehicle_vel"].type])

    def get_imu_full_data(self, idx=0):
        """``(m, 7)`` ``t wx wy wz ax ay az`` (v1 layout) of the IMU samples in the
        frame's window (as odometry); time in seconds. Orientation is in
        ``get_imu_orientation_rad``."""
        self._stream("imu")
        cols = [_IMU_COLS.index(c) for c in ("wx", "wy", "wz", "ax", "ay", "az")]
        return self._rows("imu", idx, cols)

    def get_imu_orientation_rad(self, idx):
        """Heading (yaw, rad in [-pi, pi]) from the quaternion of the newest IMU
        sample at or before the frame (v1 returned one heading per frame); nan
        when there is none yet."""
        self._stream("imu")
        j = self.sample_index("imu", idx)
        if j < 0:
            return float("nan")
        row = self.streams["imu"]._g["data"][j]
        x, y, z, w = (row[_IMU_COLS.index(c)] for c in ("qx", "qy", "qz", "qw"))
        return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def get_vector3(self, idx=0):
        """``(m, 4)`` ``t x y z`` of the Vector3Stamped samples in the frame's window."""
        self._stream("vector3")
        return self._rows("vector3", idx)

    # --- Vicon ----------------------------------------------------------------
    @property
    def vicon_frames(self):
        """``{stream name: (frame_id, child_frame_id)}`` of the TransformStamped
        (Vicon) streams; ``None`` where the stream did not record one."""
        out = {}
        for n in self.vicon_names:
            c = self.streams[n].attrs.get("child_frame_id")
            out[n] = (self.streams[n].frame_id,
                      c.decode() if isinstance(c, bytes) else (str(c) if c is not None else None))
        return out

    def _vicon_name(self, name):
        if not self.vicon_names:
            raise ValueError("no TransformStamped (vicon) stream in %s" % self.dataset_path)
        if name is None:
            return None
        if name in self.vicon_names:
            return name
        hit = [n for n, (_, c) in self.vicon_frames.items() if c == name]
        if len(hit) == 1:
            return hit[0]
        raise KeyError("vicon %r: no stream or unique child_frame_id of that name (have: %s)"
                       % (name, self.vicon_frames))

    def get_vicon_data(self, idx=0, name=None, wxyz=False):
        """Vicon / TransformStamped poses: ``(m, 7)`` ``x y z qx qy qz qw``
        (metres; quaternion x-first as stored), the rows in the frame's window
        (as odometry); ``wxyz=True`` gives v1's order ``x y z qw qx qy qz``.
        One stream: that array. Several (``vicon_<segment>``): a dict keyed by
        stream name unless ``name`` picks one (stream name or its
        ``child_frame_id``). The frame is not in the array: see ``vicon_frames``
        (frame_id, child_frame_id) per stream."""
        n = self._vicon_name(name)
        order = [0, 1, 2, 6, 3, 4, 5] if wxyz else None
        get = lambda k: self._rows(k, idx, order, with_time=False)
        if n is not None:
            return get(n)
        if len(self.vicon_names) == 1:
            return get(self.vicon_names[0])
        return {k: get(k) for k in self.vicon_names}

    # --- Leap hands and images ------------------------------------------------
    def _hand_role(self, hand):
        if hand in ("left", "right"):
            return "hand_" + hand
        if hand is not None:
            raise ValueError("hand must be None, 'left' or 'right', got %r" % (hand,))
        bound = [r for r in ("hand", "hand_left", "hand_right") if r in self.streams]
        if len(bound) > 1:
            raise ValueError("both hands are bound (%s); pass hand='left' or hand='right'"
                             % ", ".join(bound))
        return bound[0] if bound else "hand"

    def _hand(self, idx, hand):
        role = self._hand_role(hand)
        self._stream(role)
        j = self.sample_index(role, idx)
        pts = np.asarray(self.streams[role].read(j)) if j >= 0 else np.empty((0, 3))
        if not len(pts):
            return np.zeros((N_HAND_POINTS, 3)), False
        pts = pts[:, :3]
        return pts, bool(np.any(pts != 0))

    def get_hand_tracking_data(self, idx, hand=None):
        """Leap hand joints, ``(21, 3)`` x y z, joint order as v1 (palm, then
        thumb, index, middle, ring, pinky: 4 joints each). ``hand`` is
        ``"left"`` / ``"right"`` for the ``hand_left`` / ``hand_right`` streams;
        None takes the only hand stream (v1-style single ``(21, 3)`` stream).
        An absent hand (the Leap saw none: zero rows, or no sample yet) is
        ``zeros((21, 3))``; ``hand_present(idx, hand)`` tells it from a real pose.

        **Frame: raw Leap sensor coordinates in metres, returned as stored
        and never moved by ``frame=``.** Streams from the Sensors v2 recorder
        carry the Leap frame as ``frame_id`` (``cpsl_human_movement/leap_link``).
        v1 ``hands`` data converted by ``convert_v1`` are the same kind of raw
        Leap-native metres, not the robot base frame, although convert_v1
        stamps them with that frame_id (a pending fix; use them as Leap
        coordinates)."""
        return self._hand(idx, hand)[0]

    def hand_present(self, idx, hand=None):
        """True when ``get_hand_tracking_data(idx, hand)`` is a real pose, False
        for the all-zero absent-hand placeholder."""
        return self._hand(idx, hand)[1]

    def _image(self, role, idx):
        self._stream(role)
        j = self.sample_index(role, idx)
        if j < 0:
            return np.empty((0, 0), np.uint8)
        return self.streams[role].read(j)

    def get_leap_motion_image_frame_left(self, idx):
        """Leap left camera image as stored (mono8 ``(H, W)``); empty ``(0, 0)``."""
        return self._image("leap_left", idx)

    def get_leap_motion_image_frame_right(self, idx):
        """Leap right camera image as stored (mono8 ``(H, W)``); empty ``(0, 0)``."""
        return self._image("leap_right", idx)

    def get_camera_depth_frame(self, idx):
        """The ``camera_depth`` image as stored (mono16 / 32FC1 ``(H, W)``);
        empty ``(0, 0)``. Bound by name only (``camera_depth=`` picks another)."""
        return self._image("camera_depth", idx)

    # --- full point-cloud columns ---------------------------------------------
    def get_point_cloud_columns(self, idx, role="lidar", drop_invalid=False):
        """Every stored column of a cloud role (``"lidar"``, ``"radar_pc"``) as
        ``{column: 1-D array}`` (e.g. Ouster ``t``, ``reflectivity``, ``ring``,
        ``ambient``, ``range`` next to x y z intensity), in the returned frame.
        Values keep the stored dtype; ``ring`` etc. are not re-cast. Organized
        clouds hold invalid returns as NaN or as an all-zero x y z point:
        ``drop_invalid=True`` removes those rows (non-finite x, y or z, or
        x = y = z = 0), the default keeps every row. Empty arrays when no
        sample yet."""
        if role not in CLOUD_ROLES:
            raise ValueError("role must be one of %s, got %r" % (CLOUD_ROLES, role))
        pts, cols = self._cloud(role, idx, full=True)
        if drop_invalid and len(pts) and all(c in cols for c in ("x", "y", "z")):
            xyz = pts[:, [cols.index(c) for c in ("x", "y", "z")]]
            keep = np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)
            pts = pts[keep]
        return {c: pts[:, k] for k, c in enumerate(cols)}

    # --- other streams ----------------------------------------------------------
    def get_camera_info(self, idx=0):
        """CameraInfo of the newest sample at or before the frame: dict with
        ``width``, ``height``, ``K`` (3, 3), ``R`` (3, 3), ``P`` (3, 4), ``D``
        (the recorded distortion coefficients, NaN padding trimmed),
        ``distortion_model``, ``frame_id``; None when there is none yet."""
        self._stream("camera_info")
        s = self.streams["camera_info"]
        j = self.sample_index("camera_info", idx)
        if j < 0:
            return None
        row = np.asarray(s._g["data"][j], np.float64)
        c = {n: k for k, n in enumerate(s.columns)}
        d = row[[c["D%d" % k] for k in range(14) if "D%d" % k in c]]
        d = d[: int(np.max(np.flatnonzero(np.isfinite(d))) + 1)] if np.isfinite(d).any() else d[:0]
        dm = s.attrs.get("distortion_model", "")
        return {"width": int(row[c["width"]]), "height": int(row[c["height"]]),
                "K": row[[c["K%d" % k] for k in range(9)]].reshape(3, 3),
                "R": row[[c["R%d" % k] for k in range(9)]].reshape(3, 3),
                "P": row[[c["P%d" % k] for k in range(12)]].reshape(3, 4), "D": d,
                "distortion_model": dm.decode() if isinstance(dm, bytes) else str(dm),
                "frame_id": s.frame_id}

    def get_chirp_meta(self, idx):
        """The ADCChirpMeta sample of the frame (newest at or before it) as
        ``{field: array}`` exactly as stored; ``{}`` when there is none yet."""
        self._stream("chirp_meta")
        j = self.sample_index("chirp_meta", idx)
        return {} if j < 0 else self.streams["chirp_meta"].read(j)

    def get_tf_message(self, idx):
        """The newest recorded /tf message at or before the frame: ``(n, 7)``
        ``x y z qx qy qz qw`` (frame names are not stored per message; use
        ``get_transform`` for tracked pairs); empty ``(0, 7)``."""
        self._stream("tf_message")
        j = self.sample_index("tf_message", idx)
        return np.empty((0, 7)) if j < 0 else np.asarray(self.streams["tf_message"].read(j))

    def get_radar_data(self, idx):
        """Deprecated v1 getter: the radar point cloud when one is bound, else the
        ADC cube. Use ``get_radar_point_cloud`` or ``get_radar_adc_data``."""
        warnings.warn("get_radar_data is deprecated; use get_radar_adc_data or "
                      "get_radar_point_cloud", DeprecationWarning, stacklevel=2)
        if "radar_pc" in self.streams:
            return self.get_radar_point_cloud(idx)
        return self.get_radar_adc_data(idx)

    # --- transforms ---------------------------------------------------------
    def _parents(self):
        out = {c: (p, _tf.quat_to_T(x, q)) for p, c, x, q in self.capture.tf_static()}
        key = {"recv": "stamp_recv_ns", "header": "stamp_header_ns"}[self.clock]
        for (p, c), g in self.capture.tf_pairs().items():
            if c not in out and "data" in g and len(g["data"]):  # a tracked pair never received has no data
                out[c] = (p, {"stamp_header_ns": g[key][:], "data": g["data"][:]})
        return out

    def _to_root(self, frame, parents):
        """[(frame, None), (parent, edge), (grandparent, edge), ...] up to the root;
        each edge is T_parent_child (array) or a tracked group (dict)."""
        chain, seen = [(frame, None)], {frame}
        while frame in parents:
            frame, e = parents[frame]
            if frame in seen:
                raise ValueError("tf cycle at %r" % frame)
            seen.add(frame)
            chain.append((frame, e))
        return chain

    @staticmethod
    def _compose(chain, n, t_ns):
        """T_<chain[n]>_<chain[0]> from the first n edges of a _to_root chain."""
        M = np.eye(4)
        for k in range(1, n + 1):
            frame, e = chain[k]
            if isinstance(e, dict):
                if t_ns is None:
                    raise ValueError("%s -> %s is a tracked /tf edge; pass idx= or t_ns="
                                     % (frame, chain[k - 1][0]))
                e = _tf.pose_at(e, t_ns)
            M = e @ M
        return M

    def get_transform(self, target, source, idx=None, t_ns=None):
        """4x4 ``T_target_source`` (maps ``source`` coordinates into ``target``)."""
        if idx is not None and t_ns is None:
            t_ns = self._ref_time(idx)[1]
        parents = self._parents()
        if not parents:
            raise ValueError("no /tf_static or /tf in %s (format v%d); transforms need v4"
                             % (self.dataset_path, self.capture.version))
        up_s = self._to_root(source, parents)
        up_t = self._to_root(target, parents)
        names_t = [f for f, _ in up_t]
        for n, (anc, _) in enumerate(up_s):
            if anc in names_t:
                m = names_t.index(anc)
                T_anc_src = self._compose(up_s, n, t_ns)
                T_anc_tgt = self._compose(up_t, m, t_ns)
                return np.linalg.inv(T_anc_tgt) @ T_anc_src
        raise ValueError("no tf path between %r and %r" % (target, source))
