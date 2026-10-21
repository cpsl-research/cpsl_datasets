"""CpslDS v2: v1-style frame getters over a cpsl_dataset_hdf5 capture (format v2-4)."""
import warnings

import numpy as np

from .hdf5 import Capture
from .hdf5 import tf as _tf

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
}
REFERENCE_ORDER = ("radar_adc", "radar_pc", "lidar", "camera")
CLOUD_ROLES = ("radar_pc", "lidar")
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
      v3+; ``metrics`` decoded) or None.
    - ``get_transform(target, source, idx=None, t_ns=None)``: 4x4
      ``T_target_source`` over ``/tf_static`` and the tracked ``/tf`` pairs
      (v4). A tracked edge is interpolated at ``t_ns``, or at the reference
      stamp of ``idx``, on the chosen clock; it raises if neither is given.

    Use as a context manager or call ``close()``.
    """

    def __init__(self, dataset_path, radar_adc=None, radar_pc=None, lidar=None,
                 camera=None, vehicle_odom=None, hand_left=None, hand_right=None,
                 hand=None, reference=None, sync=True,
                 clock="recv", frame=None):
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
                             hand=hand), reference)
        except Exception:
            self.capture.close()
            raise

    # --- setup ----------------------------------------------------------
    def _setup(self, requested, reference):
        all_streams = self.capture.streams()
        self.streams = {}
        for role, name in requested.items():
            s = self._bind(role, name, all_streams)
            if s is not None:
                self.streams[role] = s
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
        if name is not None:
            if name not in all_streams:
                raise KeyError("%s=%r: no such stream in %s (streams: %s)"
                               % (role, name, self.dataset_path, sorted(all_streams)))
            return all_streams[name]
        hits = [s for s in all_streams.values()
                if s.type == mtype and (pred is None or pred(s))]
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
        return self.capture.radar_cfg(s.name if s is not None else None)

    def get_radar_adc_data(self, idx):
        j = self.sample_index("radar_adc", idx)
        if j < 0:
            return np.empty((0, 0, 0), np.complex64)
        return self.streams["radar_adc"].read(j)

    def _cloud(self, role, idx):
        """(points, column names) of a cloud role, frame applied."""
        s = self._stream(role)
        j = self.sample_index(role, idx)
        cols = list(s.columns)
        want = ROLES[role][2]
        sel = [cols.index(c) for c in want] if all(c in cols for c in want) else None
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

    def get_vehicle_odom_data(self, idx=0):
        s = self._stream("vehicle_odom")
        t = self._t["vehicle_odom"]
        if not self.sync:
            j = s._index(idx)
            rows = np.array([j])
        else:
            i, t1 = self._ref_time(idx)
            if self.reference == "vehicle_odom":
                rows = np.array([i])
            else:
                lo = int(self._t[self.reference][i - 1]) if i > 0 else None
                m = t <= t1 if lo is None else (t > lo) & (t <= t1)
                rows = np.flatnonzero(m)
                rows = rows[np.argsort(t[rows], kind="stable")]
        if not len(rows):
            return np.empty((0, ODOM_COLS))
        srt = np.sort(rows)
        data = s._g["data"][srt][np.searchsorted(srt, rows)]
        return np.c_[t[rows] * 1e-9, data]

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
