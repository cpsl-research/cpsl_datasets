"""Read-only access to a cpsl_dataset_hdf5 capture (format v2-4), no ROS."""
import json
import warnings

import h5py
import numpy as np

from . import tf as _tf

NON_STREAMS = ("tf", "tf_static")


def attr_str(v):
    """An HDF5 attribute as str (bytes/np.str_ handled)."""
    if isinstance(v, bytes):
        return v.decode("utf-8", "replace")
    return str(v)


def attr_list(v):
    """A list-valued attribute (object array of str) as list[str]."""
    if v is None:
        return []
    if isinstance(v, (str, bytes)):
        return [attr_str(v)]
    return [attr_str(x) for x in np.asarray(v).ravel()]


def short_type(msg_type):
    """``raw_radar_msgs/msg/ADCDataCube`` -> ``ADCDataCube``."""
    return attr_str(msg_type).rsplit("/", 1)[-1]


class Stream:
    """One stream group. ``s[i]`` decodes sample i; ``len(s)`` is its count."""

    def __init__(self, group, name=None):
        self._g = group
        self.name = name or group.name.lstrip("/")
        self.attrs = group.attrs
        self.msg_type = attr_str(group.attrs.get("msg_type", ""))
        self.type = short_type(self.msg_type)
        fid = group.attrs.get("frame_id")
        self.frame_id = attr_str(fid) if fid is not None else None
        self.columns = attr_list(group.attrs.get("columns"))
        self._off = None

    @property
    def mode(self):
        """How the stream was recorded: ``"all"`` (every message) or
        ``"sampled"`` (the newest unseen message per tick of the capture's
        ``sample_rate_hz``). The stream's own ``mode`` attr, else the root
        ``capture_mode``; None when the file records neither (v2/v3)."""
        v = self.attrs.get("mode")
        if v is None:
            v = self._g.file.attrs.get("capture_mode")
        return attr_str(v) if v is not None else None

    def __len__(self):
        if "stamp_recv_ns" in self._g:
            return int(self._g["stamp_recv_ns"].shape[0])
        return int(self._g.attrs.get("n_samples", 0))

    def __repr__(self):
        return "<Stream %s %s n=%d>" % (self.name, self.type, len(self))

    def stamps(self, clock="recv"):
        """int64 stamps in ns. clock='recv' (host, common to all streams) or
        'header' (sensor clock; -1 where the message had no header)."""
        key = {"recv": "stamp_recv_ns", "header": "stamp_header_ns"}.get(clock)
        if key is None:
            raise ValueError("clock must be 'recv' or 'header', got %r" % (clock,))
        if key not in self._g:
            return np.zeros(0, np.int64)
        return self._g[key][:]

    @property
    def ragged(self):
        return "row_count" in self._g

    def _offsets(self):
        if self._off is None:
            self._off = np.concatenate([[0], np.cumsum(self._g["row_count"][:])])
        return self._off

    def _index(self, i):
        n = len(self)
        i = int(i)
        if i < 0:
            i += n
        if not 0 <= i < n:
            raise IndexError("%s: sample %d out of range (%d)" % (self.name, i, n))
        return i

    def read(self, i, raw=False):
        """Decode sample ``i``. ``raw=True`` keeps an ADC cube as int16
        ``(2, rx, sample, chirp)`` instead of complex ``(rx, sample, chirp)``."""
        i = self._index(i)
        g = self._g
        if self.type == "ADCChirpMeta":
            return {k: g[k][i] for k in g
                    if k not in ("stamp_header_ns", "stamp_recv_ns")
                    and isinstance(g[k], h5py.Dataset)}
        if self.ragged:
            if "data" not in g:
                return np.zeros((0, len(self.columns)))
            off = self._offsets()
            return g["data"][int(off[i]):int(off[i + 1])]
        d = g["data"][i]
        if self.type == "ADCDataCube" and not raw:
            d = d.astype(np.float32)
            return d[0] + 1j * d[1]
        return d

    __getitem__ = read

    def named(self, i):
        """Row-type samples as ``{column: values}`` (ragged: arrays per column)."""
        a = self.read(i)
        if not self.columns or not isinstance(a, np.ndarray):
            return a
        if a.ndim == 1:
            return {c: a[k] for k, c in enumerate(self.columns)}
        return {c: a[:, k] for k, c in enumerate(self.columns)}


class Capture:
    """Opens a capture read-only.

    Attributes: ``version``, ``closed_cleanly``, ``warnings`` (list of str),
    ``attrs`` (root attrs), ``path``, ``capture_mode``, ``sample_rate_hz``.
    Use as a context manager or call close().
    """

    def __init__(self, path):
        self.path = str(path)
        self._f = h5py.File(self.path, "r")
        a = self._f.attrs
        self.attrs = a
        self.format = attr_str(a.get("format", ""))
        self.version = int(a.get("format_version", 0))
        self.closed_cleanly = bool(a.get("closed_cleanly", False))
        self.warnings = []
        if self.format != "cpsl_dataset_hdf5":
            self.warnings.append("format is %r, not 'cpsl_dataset_hdf5'" % self.format)
        if not self.closed_cleanly:
            msg = ("closed_cleanly is False: the recorder died; "
                   "samples up to the last flush are readable")
            self.warnings.append(msg)
            warnings.warn("%s: %s" % (self.path, msg), stacklevel=2)

    def close(self):
        if self._f:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def capture_mode(self):
        """Root ``capture_mode``: ``"all"`` (every message is a row) or
        ``"sampled"`` (one row per tick at most, the newest unseen message;
        the tick time itself is not stored). None when the file has no such
        attr (format v2/v3). A stream may override it: ``Stream.mode``."""
        v = self.attrs.get("capture_mode")
        return attr_str(v) if v is not None else None

    @property
    def sample_rate_hz(self):
        """Root ``sample_rate_hz`` (float), the tick rate of ``sampled``
        streams; None when absent (e.g. files converted before the converter
        wrote it). The recorder writes it even when every stream is ``all``,
        where it means nothing."""
        v = self.attrs.get("sample_rate_hz")
        return float(v) if v is not None else None

    @property
    def h5(self):
        """The underlying h5py.File (read-only)."""
        return self._f

    def streams(self):
        """``{name: Stream}`` discovered from the file (tf, tf_static excluded)."""
        return {n: Stream(self._f[n], n) for n in self._f
                if n not in NON_STREAMS and isinstance(self._f[n], h5py.Group)}

    def __getitem__(self, name):
        return self.streams()[name]

    @property
    def robot_description(self):
        v = self._f.attrs.get("robot_description")
        return attr_str(v) if v is not None else None

    def radar_cfg(self, stream=None):
        """Parsed radar cfg attrs of a cube stream (first one that has them if
        ``stream`` is None), or None. ``metrics`` is the decoded JSON."""
        names = [stream] if stream else list(self.streams())
        for n in names:
            a = self._f[n].attrs
            if "radar_cfg_status" not in a:
                continue
            out = {k[len("radar_cfg_"):]: a[k] for k in a if k.startswith("radar_cfg_")}
            for k, v in list(out.items()):
                out[k] = attr_str(v) if isinstance(v, (bytes, str, np.str_)) else v
            m = out.get("metrics")
            if m:
                out["metrics"] = json.loads(m)
            return out
        return None

    # --- transforms (format v4) -------------------------------------------
    def tf_static(self):
        """List of (parent, child, xyz, quat_xyzw); [] when there is none."""
        return _tf.static_edges(self._f)

    def tf_pairs(self):
        """``{(parent, child): group}`` of tracked /tf pairs."""
        out = {}
        if "tf" in self._f:
            for k, g in self._f["tf"].items():
                p = attr_str(g.attrs.get("parent_frame", k.split("__")[0]))
                c = attr_str(g.attrs.get("child_frame", k.split("__")[-1]))
                out[(p, c)] = g
        return out

    def static_chain(self, frame, up_to):
        return _tf.static_chain(self._f, frame, up_to)

    def pose_at(self, parent, child, t_ns, interpolate=True):
        return _tf.pose_at(self.tf_pairs()[(parent, child)], t_ns, interpolate)
