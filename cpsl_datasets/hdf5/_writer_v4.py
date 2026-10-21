# Copied verbatim (everything below this header) from CPSL_ROS2_Sensors:
#   src/dataset_generator/dataset_generator/hdf5_writer.py
# Source commit (last commit touching that file): f26e546777907a9163f35ef804b5c9f2d334239c
# Sensors repo HEAD at copy time: f0b256a88d7c35e4fdd7b64ac07bb9a2e5b14764
# Writes format_version 4. Do not edit here: re-copy when the format changes.
"""HDF5 capture file writer (no ROS imports, so it is unit-testable).

One file per capture. Every stream is a group ``/<stream>`` holding

* ``stamp_header_ns``  int64 (N,)  the message header stamp (-1 if unstamped)
* ``stamp_recv_ns``    int64 (N,)  host wall-clock time the message was received
* ``data`` (and other named fields)  one row per sample, chunked + resizable
* ``row_count`` (ragged fields only)  rows of ``data`` belonging to each sample

See docs/hdf5_format.md for the full layout.
"""
import json
import signal
import threading
import time

import h5py
import numpy as np

FORMAT_NAME = 'cpsl_dataset_hdf5'
FORMAT_VERSION = 4
_CHUNK_BYTES = 256 * 1024


def _scalar_attr(v):
    """Make a value storable as an HDF5 attribute."""
    if isinstance(v, (dict,)):
        return json.dumps(v, sort_keys=True)
    if isinstance(v, (list, tuple)):
        if all(isinstance(x, str) for x in v):
            return np.array(v, dtype=h5py.string_dtype())
        return np.asarray(v)
    if v is None:
        return ''
    return v


class _Stream:
    def __init__(self, name, attrs, compression, keep_stamps):
        self.name = name
        self.attrs = dict(attrs)
        self.compression = compression
        self.keep_stamps = keep_stamps
        self.n = 0
        self.n_errors = 0
        self.header_stamps = [] if keep_stamps else None
        self.ragged_fields = set()
        self.fields = {}      # field name -> h5py dataset
        self.grp = None
        # frame_index tracking (any stream that carries one)
        self.fi_first = None
        self.fi_last = None
        self.fi_gaps = 0
        self.fi_nonmono = 0


class CaptureFile:
    """Append-only HDF5 capture, flushed periodically."""

    def __init__(self, path, attrs=None, flush_period_s=5.0):
        self.path = str(path)
        self.flush_period_s = flush_period_s
        self._f = h5py.File(self.path, 'w')
        self._streams = {}
        self._last_flush = time.monotonic()
        self._closed = False
        root = self._f.attrs
        root['format'] = FORMAT_NAME
        root['format_version'] = FORMAT_VERSION
        root['closed_cleanly'] = False
        root['start_recv_ns'] = time.time_ns()
        for k, v in (attrs or {}).items():
            root[k] = _scalar_attr(v)
        self._f.flush()

    # ---- setup -------------------------------------------------------
    def add_stream(self, name, attrs=None, compression=None,
                   keep_stamps=False):
        # 'tf/<parent>__<child>' is the one nested name (tracked /tf pairs);
        # frame names may carry '.' there ('/' was replaced by the caller).
        nested = name.startswith('tf/')
        body = name[3:] if nested else name
        if not body or '/' in body or (not nested and '.' in body):
            raise ValueError('bad stream name %r' % name)
        if name in self._streams:
            raise ValueError('duplicate stream %r' % name)
        s = _Stream(name, attrs or {}, compression, keep_stamps)
        s.grp = self._f.create_group(name)
        for k, v in s.attrs.items():
            s.grp.attrs[k] = _scalar_attr(v)
        for stamp in ('stamp_header_ns', 'stamp_recv_ns'):
            s.grp.create_dataset(stamp, shape=(0,), maxshape=(None,),
                                 dtype=np.int64, chunks=(1024,))
        self._streams[name] = s
        return s

    def write_tf_static(self, rows):
        """
        (Re)write group ``/tf_static`` from the latest transform per pair.

        ``rows``: iterable of (parent, child, xyz, quat_xyzw, stamp_ns,
        recv_ns). Replaces the previous group, so it can be called again when
        a new pair arrives and once more at close. Call from the thread that
        owns the file.
        """
        rows = list(rows)
        n = len(rows)
        if 'tf_static' in self._f:
            del self._f['tf_static']
        g = self._f.create_group('tf_static')
        sd = h5py.string_dtype()
        for key, col in (('parent', 0), ('child', 1)):
            ds = g.create_dataset(key, shape=(n,), dtype=sd)
            for i, r in enumerate(rows):
                ds[i] = r[col]
        g.create_dataset('xyz', data=np.asarray(
            [r[2] for r in rows], np.float64).reshape(n, 3))
        g.create_dataset('quat_xyzw', data=np.asarray(
            [r[3] for r in rows], np.float64).reshape(n, 4))
        g.create_dataset('stamp_ns', data=np.asarray(
            [r[4] for r in rows], np.int64).reshape(n))
        g.create_dataset('recv_ns', data=np.asarray(
            [r[5] for r in rows], np.int64).reshape(n))
        g.attrs['n_transforms'] = n

    def streams(self):
        return list(self._streams)

    def count(self, name):
        return self._streams[name].n

    # ---- writing -----------------------------------------------------
    def _make_field(self, s, key, arr, ragged):
        arr = np.asarray(arr)
        tail = arr.shape[1:] if ragged else arr.shape
        row_bytes = max(1, int(np.prod(tail, dtype=np.int64)) * arr.dtype.itemsize)
        rows = max(1, min(1024, _CHUNK_BYTES // row_bytes))
        if not ragged:
            rows = max(1, min(rows, 1024))
        kwargs = {}
        if s.compression and arr.dtype != object:
            kwargs['compression'] = s.compression
        ds = s.grp.create_dataset(
            key, shape=(0,) + tuple(tail), maxshape=(None,) + tuple(tail),
            dtype=arr.dtype, chunks=(rows,) + tuple(tail), **kwargs)
        s.fields[key] = ds
        if ragged:
            s.ragged_fields.add(key)
            if 'row_count' not in s.fields:
                s.fields['row_count'] = s.grp.create_dataset(
                    'row_count', shape=(0,), maxshape=(None,),
                    dtype=np.int64, chunks=(1024,))

    @staticmethod
    def _grow(ds, add):
        n = ds.shape[0]
        ds.resize(n + add, axis=0)
        return n

    def append(self, name, header_ns, recv_ns, fields, ragged=()):
        """Append one sample. ``fields`` maps field name -> ndarray.

        A shape/dtype mismatch against the stream's first sample raises
        ValueError and writes nothing (the stream's n_errors is bumped).
        """
        s = self._streams[name]
        fields = {k: np.asarray(v) for k, v in fields.items()}
        # validate everything before touching the file
        for k, arr in fields.items():
            ds = s.fields.get(k)
            if ds is None:
                continue
            tail = ds.shape[1:]
            got = arr.shape[1:] if k in s.ragged_fields else arr.shape
            if tuple(got) != tuple(tail) or arr.dtype != ds.dtype:
                s.n_errors += 1
                raise ValueError(
                    '%s.%s: sample %s %s does not match stream %s %s' %
                    (name, k, arr.shape, arr.dtype, tuple(tail), ds.dtype))
        for k, arr in fields.items():
            if k not in s.fields:
                self._make_field(s, k, arr, k in ragged)
        for k, arr in fields.items():
            ds = s.fields[k]
            if k in s.ragged_fields:
                n = self._grow(ds, arr.shape[0])
                if arr.shape[0]:
                    ds[n:n + arr.shape[0]] = arr
                rc = s.fields['row_count']
                rn = self._grow(rc, 1)
                rc[rn] = arr.shape[0]
            else:
                n = self._grow(ds, 1)
                ds[n] = arr
        for key, val in (('stamp_header_ns', header_ns),
                         ('stamp_recv_ns', recv_ns)):
            ds = s.grp[key]
            n = self._grow(ds, 1)
            ds[n] = val
        s.n += 1
        if s.keep_stamps:
            s.header_stamps.append(int(header_ns))
        if 'frame_index' in fields:
            self._track_frame_index(s, int(fields['frame_index']))
        if time.monotonic() - self._last_flush >= self.flush_period_s:
            self.flush()

    @staticmethod
    def _track_frame_index(s, idx):
        if s.fi_first is None:
            s.fi_first = s.fi_last = idx
            return
        delta = (idx - s.fi_last) & 0xFFFFFFFF   # uint32 wrap-safe
        if delta == 0 or delta > 0x7FFFFFFF:
            s.fi_nonmono += 1
            return
        s.fi_gaps += delta - 1
        s.fi_last = idx

    # ---- bookkeeping ---------------------------------------------------
    def cross_check(self, meta_stream, cube_stream):
        """Match two keep_stamps streams on header stamp (cube vs adc_meta).

        Writes, on the cube stream: ``meta_stream``, ``n_without_meta`` and,
        when the meta stream carries frame_index, ``frame_index_published``
        (span first..last of the meta, a lower bound if the meta itself lost
        messages), ``frame_index_recorded`` (cubes written) and
        ``frame_index_gap`` = published - recorded. On the meta stream:
        ``cube_stream``, ``n_without_cube``.
        """
        m = self._streams.get(meta_stream)
        c = self._streams.get(cube_stream)
        if m is None or c is None or not m.keep_stamps or not c.keep_stamps:
            return None
        ms, cs = set(m.header_stamps), set(c.header_stamps)
        res = {'n_without_meta': len(cs - ms), 'n_without_cube': len(ms - cs)}
        c.attrs['meta_stream'] = meta_stream
        c.attrs['n_without_meta'] = res['n_without_meta']
        m.attrs['cube_stream'] = cube_stream
        m.attrs['n_without_cube'] = res['n_without_cube']
        if m.fi_first is not None:
            pub = ((m.fi_last - m.fi_first) & 0xFFFFFFFF) + 1
            c.attrs['frame_index_published'] = pub
            c.attrs['frame_index_recorded'] = c.n
            c.attrs['frame_index_gap'] = pub - c.n
            res.update(published=pub, recorded=c.n, gap=pub - c.n)
        return res

    def update_stream_attrs(self, name, attrs):
        self._streams[name].attrs.update(attrs)

    def set_root_attr(self, key, value):
        self._f.attrs[key] = _scalar_attr(value)

    def _write_stream_attrs(self):
        for s in self._streams.values():
            a = s.grp.attrs
            a['n_samples'] = s.n
            a['n_errors'] = s.n_errors
            for k, v in s.attrs.items():
                a[k] = _scalar_attr(v)
            if s.fi_first is not None:
                a['frame_index_first'] = s.fi_first
                a['frame_index_last'] = s.fi_last
                a['frame_index_recorded_gaps'] = s.fi_gaps
                a['frame_index_nonmonotonic'] = s.fi_nonmono

    def flush(self):
        if self._closed:
            return
        self._write_stream_attrs()
        self._f.attrs['last_flush_recv_ns'] = time.time_ns()
        self._f.flush()
        self._last_flush = time.monotonic()

    def close(self):
        if self._closed:
            return
        # Once closing starts, SIGINT/SIGTERM are ignored until the file is
        # flushed and closed_cleanly is written: a KeyboardInterrupt inside an
        # h5py attribute write would leave closed_cleanly False.
        saved = {}
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    saved[sig] = signal.signal(sig, signal.SIG_IGN)
                except (ValueError, OSError):
                    pass
        try:
            self._write_stream_attrs()
            self._f.attrs['end_recv_ns'] = time.time_ns()
            self._f.attrs['closed_cleanly'] = True
            self._f.flush()
            self._f.close()
            self._closed = True
        finally:
            for sig, h in saved.items():
                signal.signal(sig, h)
