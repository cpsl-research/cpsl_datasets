"""Convert a v1 folder-of-``frame_N.npy`` dataset to one cpsl_dataset_hdf5 v4 file.

    python -m cpsl_datasets.convert_v1 V1_DIR OUT.h5 [--map folder=type ...]
        [--base-frame FRAME] [--hands-frame FRAME] [--frame-rate HZ] [--compression gzip|lzf] [--force]

The source tree is only ever read. Each folder becomes one stream (attr
``legacy_folder`` keeps the folder name; stream names follow the Sensors
recorder configs via ``RENAMES``, e.g. depth -> camera_depth, radar_0_adc ->
radar_adc, leap_images_left -> leap_left); folders no pattern knows are printed
and listed in root attr ``v1_skipped_folders``. Folder names vary between
datasets, so the folder -> type mapping is by name pattern (``PATTERNS``) and
can be overridden per folder with ``--map folder=type`` (``skip`` drops one).

Types and conversions (reading_datasets.md section 5):

* ``cube``        complex (rx,sample,chirp) -> int16 (N,2,rx,sample,chirp);
                  non-integer or out-of-int16-range values abort the run.
* ``radar_pc``    (n,4) x y z vel, ragged PointCloud2, frame_id = --base-frame.
* ``lidar``       (n,4) x y z intensity, same.
* ``hands``       (2,n,3) -> two ragged streams (folder ``hands`` ->
                  ``hand_left``/``hand_right``, else ``<folder>_left/_right``);
                  (21,3) -> one stream. x y z, raw Leap metres, frame_id = --hands-frame
                  (default cpsl_human_movement/leap_link; not the base frame).
* ``image``       png (cv2, BGR) or npy -> Image; encoding from dtype/channels.
* ``imu_orient``  (4,) w x y z -> Imu (N,10), orientation only (rest NaN).
* ``imu_full``    (m,7) t wx wy wz ax ay az -> Imu (N,10), orientation NaN.
* ``vel``         (m,3) t vx wz -> TwistStamped vx 0 0 0 0 wz.
* ``odom``        (m,14) t x y z qw qx qy qz v.. -> Odometry, quaternion x-first.
* ``pose``        (7,) x y z qw qx qy qz -> TransformStamped, x-first.

Time: v1 stored no header stamp (``stamp_header_ns = -1``). ``stamp_recv_ns`` of
row streams (imu_full, odom, vel) is the row's ``t`` (node clock, s) * 1e9. A
per-frame stream's frame k takes the last ``t`` of frame k in the first of the
odom, vel, imu_full folders that has it; otherwise ``k / --frame-rate``
(default 10 Hz) seconds. The method is recorded in root attr ``timestamps``.
"""
import argparse
import datetime
import fnmatch
import json
import os
import re
import sys
from pathlib import Path

import cv2
import numpy as np

from .hdf5._writer_v4 import CaptureFile

CONVERTER_VERSION = "1"
FRAME_RE = re.compile(r"^frame_(\d+)\.(npy|png)$")

# (fnmatch pattern on the lower-cased folder name, type); first match wins.
PATTERNS = [
    ("*_adc", "cube"),
    ("*_pc", "radar_pc"),
    ("lidar*", "lidar"),
    ("imu_data_full*", "imu_full"),
    ("imu_data*", "imu_orient"),
    ("vehicle_vel*", "vel"),
    ("vehicle_odom*", "odom"),
    ("vicon_*", "pose"),
    ("hand*", "hands"),
    ("camera*", "image"),
    ("*depth*", "image"),
    ("leap*", "image"),
]
TYPES = {"cube", "radar_pc", "lidar", "hands", "image", "imu_orient",
         "imu_full", "vel", "odom", "pose", "skip"}
TIME_SOURCES = ("odom", "vel", "imu_full")  # priority for per-frame stamps

MSG = {
    "cube": "raw_radar_msgs/msg/ADCDataCube",
    "cloud": "sensor_msgs/msg/PointCloud2",
    "image": "sensor_msgs/msg/Image",
    "imu": "sensor_msgs/msg/Imu",
    "odom": "nav_msgs/msg/Odometry",
    "vel": "geometry_msgs/msg/TwistStamped",
    "pose": "geometry_msgs/msg/TransformStamped",
}
ODOM_COLS = "x y z qx qy qz qw vx vy vz wx wy wz".split()
IMU_COLS = "qx qy qz qw wx wy wz ax ay az".split()
TWIST_COLS = "vx vy vz wx wy wz".split()
POSE_COLS = "x y z qx qy qz qw".split()
ENCODINGS = {  # (dtype, ndim, channels) -> encoding
    (np.uint8, 2, None): "mono8", (np.uint8, 3, 3): "bgr8",
    (np.uint8, 3, 4): "bgra8", (np.uint16, 2, None): "mono16",
    (np.float32, 2, None): "32FC1",
}
INT16 = np.iinfo(np.int16)


class ConvertError(Exception):
    """A conversion that must not continue (bad data, unsafe paths)."""


class Folder:
    def __init__(self, name, kind, files):
        self.name = name
        self.kind = kind
        self.files = files  # [(frame number, Path)] ascending by number


# v1 folder -> stream name the Sensors v2 recorder configs use (so converted
# and newly captured files read alike). Regexes on the folder name; first hit.
# Left alone (no recorder convention): radar_1_*/radar_combined_pc, imu_data_full,
# vehicle_vel, vicon_<segment>, single-hand "hands" (21,3), camera_npy, others.
RENAMES = [
    (r"radar_0_(adc|pc)", r"radar_\1"),
    (r"(front|back|down)_radar_(adc|pc)", r"radar_\1_\2"),
    (r"imu_data", "imu"),
    (r"depth", "camera_depth"),
    (r"leap_images_(left|right)", r"leap_\1"),
]
HANDS_SUFFIX = {"hands": ("hand_left", "hand_right")}


def stream_name(folder):
    name = folder.replace(".", "_")
    for pat, rep in RENAMES:
        if re.fullmatch(pat, name):
            name = re.sub(pat, rep, name)
            break
    if name in ("tf", "tf_static"):
        raise ConvertError("folder %r collides with reserved stream name" % folder)
    return name


def parse_maps(items):
    out = {}
    for it in items or []:
        if "=" not in it:
            raise ConvertError("--map expects folder=type, got %r" % it)
        k, v = it.split("=", 1)
        if v not in TYPES:
            raise ConvertError("--map %s: unknown type %r (one of %s)" % (
                k, v, ", ".join(sorted(TYPES))))
        out[k] = v
    return out


def discover(v1_dir, overrides=None):
    """Return (folders, skipped): mapped Folders and unmapped folder names."""
    overrides = overrides or {}
    v1_dir = Path(v1_dir)
    folders, skipped = [], []
    subdirs = sorted(p for p in v1_dir.iterdir() if p.is_dir())
    names = {p.name for p in subdirs}
    for k in overrides:
        if k not in names:
            raise ConvertError("--map %s: no such folder in %s" % (k, v1_dir))
    for p in subdirs:
        kind = overrides.get(p.name)
        if kind is None:
            kind = next((t for pat, t in PATTERNS
                         if fnmatch.fnmatch(p.name.lower(), pat)), None)
        if kind is None or kind == "skip":
            skipped.append(p.name)
            continue
        files, odd = [], []
        for f in os.scandir(p):
            m = FRAME_RE.match(f.name)
            if m:
                files.append((int(m.group(1)), Path(f.path)))
            else:
                odd.append(f.name)
        if odd:
            print("note: %s: ignoring %d file(s) not named frame_N.npy/png (e.g. %s)"
                  % (p.name, len(odd), odd[0]))
        files.sort(key=lambda t: t[0])
        folders.append(Folder(p.name, kind, files))
    return folders, skipped


def _load(path):
    if path.suffix == ".png":
        a = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if a is None:
            raise ConvertError("cannot read image %s" % path)
        return a
    return np.load(path, allow_pickle=False)


def cube_to_int16(a, where):
    """complex (rx,sample,chirp) -> int16 (2,rx,sample,chirp); never rounds."""
    a = np.asarray(a)
    if a.ndim != 3:
        raise ConvertError("%s: cube must be (rx,sample,chirp), got %s" % (where, a.shape))
    ri = np.stack([a.real, a.imag]).astype(np.float64) if np.iscomplexobj(a) \
        else np.stack([a, np.zeros_like(a)]).astype(np.float64)
    if not np.all(np.isfinite(ri)) or not np.array_equal(ri, np.rint(ri)):
        raise ConvertError("%s: ADC cube has non-integer values; refusing to round" % where)
    if ri.min() < INT16.min or ri.max() > INT16.max:
        raise ConvertError("%s: ADC cube value outside int16 range (%d..%d)" % (
            where, ri.min(), ri.max()))
    return ri.astype(np.int16)


def reorder_wxyz(q):
    """w-first quaternion(s), last axis 4 -> x-first."""
    return np.asarray(q)[..., [1, 2, 3, 0]]


def _rows(a, k, where):
    a = np.asarray(a, np.float64)
    if a.size % k:
        raise ConvertError("%s: expected rows of %d values, got shape %s" % (where, k, a.shape))
    return a.reshape(-1, k)


def frame_stamps(folders, frame_rate):
    """{frame number: ns} and the text for root attr ``timestamps``."""
    by_kind = {f.kind: f for f in folders}
    sources = [by_kind[k] for k in TIME_SOURCES if k in by_kind and by_kind[k].files]
    nums = sorted({n for f in folders for n, _ in f.files})
    out, used, synth = {}, set(), 0
    base = nums[0] if nums else 0
    for n in nums:
        ns = None
        for src in sources:
            hit = next((p for m, p in src.files if m == n), None)
            if hit is None:
                continue
            a = _rows(_load(hit), {"odom": 14, "vel": 3, "imu_full": 7}[src.kind], str(hit))
            if len(a):
                ns = int(np.rint(a[-1, 0] * 1e9))
                used.add(src.name)
                break
        if ns is None:
            ns = int(round((n - base) / frame_rate * 1e9))
            synth += 1
        out[n] = ns
    parts = ["row streams: stamp_recv_ns = v1 t column (node clock, s) * 1e9; "
             "stamp_header_ns = -1 (v1 stored no header stamp)"]
    if used:
        parts.append("per-frame streams: last t of the frame in %s" % ", ".join(sorted(used)))
    if synth:
        parts.append("%d of %d frames synthesized as k/%g Hz from the first frame"
                     % (synth, len(nums), frame_rate))
    return out, "; ".join(parts)


def _image_encoding(a, where):
    ch = a.shape[2] if a.ndim == 3 else None
    enc = ENCODINGS.get((a.dtype.type, a.ndim, ch))
    if enc is None:
        raise ConvertError("%s: unsupported image dtype/shape %s %s" % (where, a.dtype, a.shape))
    return enc


HANDS_FRAME = "cpsl_human_movement/leap_link"


class _Ctx:
    def __init__(self, cap, base_frame, stamps, compression, hands_frame=HANDS_FRAME):
        self.cap, self.base_frame = cap, base_frame
        self.hands_frame = hands_frame
        self.stamps, self.compression = stamps, compression
        self.streams = []  # (stream name, folder, n)

    def add(self, name, folder, msg, attrs):
        a = {"topic": folder.name, "msg_type": msg, "mode": "sampled",
             "legacy_folder": folder.name, "v1_files": len(folder.files)}
        a.update(attrs)
        self.cap.add_stream(name, a, compression=self.compression)

    def put(self, name, folder, num, hdr, rows, fields, ragged=()):
        try:
            self.cap.append(name, hdr, rows, fields, ragged=ragged)
        except ValueError as e:
            raise ConvertError("%s frame_%d: %s" % (folder.name, num, e)) from e


def _convert_folder(ctx, f):
    sname = stream_name(f.name)
    k = f.kind
    if k == "cube":
        ctx.add(sname, f, MSG["cube"], {
            "columns": ["real", "imag"], "axes": ["re_im", "rx", "sample", "chirp"],
            "radar_cfg_status": "not_received"})
        for num, p in f.files:
            d = cube_to_int16(_load(p), str(p))
            ctx.put(sname, f, num, -1, ctx.stamps[num], {"data": d})
    elif k in ("radar_pc", "lidar"):
        cols = ["x", "y", "z", "vel" if k == "radar_pc" else "intensity"]
        ctx.add(sname, f, MSG["cloud"], {"columns": cols, "frame_id": ctx.base_frame})
        for num, p in f.files:
            d = _rows(_load(p), 4, str(p)).astype(np.float32)
            ctx.put(sname, f, num, -1, ctx.stamps[num], {"data": d}, ("data",))
    elif k == "hands":
        _convert_hands(ctx, f, sname)
    elif k == "image":
        enc = None
        for num, p in f.files:
            a = _load(p)
            if enc is None:
                enc = _image_encoding(a, str(p))
                ctx.add(sname, f, MSG["image"], {"encoding": enc})
            ctx.put(sname, f, num, -1, ctx.stamps[num], {"data": a})
        if enc is None:
            ctx.add(sname, f, MSG["image"], {"encoding": ""})
    elif k == "imu_orient":
        ctx.add(sname, f, MSG["imu"], {"columns": IMU_COLS, "frame_id": ctx.base_frame})
        for num, p in f.files:
            q = np.asarray(np.load(p, allow_pickle=False), np.float64).reshape(-1)
            if q.shape != (4,):
                raise ConvertError("%s: imu_data must be (4,) w x y z, got %s" % (p, q.shape))
            row = np.full(10, np.nan)
            row[:4] = reorder_wxyz(q)
            ctx.put(sname, f, num, -1, ctx.stamps[num], {"data": row})
    elif k == "imu_full":
        ctx.add(sname, f, MSG["imu"], {"columns": IMU_COLS, "frame_id": ctx.base_frame})
        for num, p in f.files:
            for r in _rows(_load(p), 7, str(p)):
                row = np.full(10, np.nan)
                row[4:] = r[1:]
                ctx.put(sname, f, num, -1, int(np.rint(r[0] * 1e9)), {"data": row})
    elif k == "vel":
        ctx.add(sname, f, MSG["vel"], {"columns": TWIST_COLS, "frame_id": ctx.base_frame})
        for num, p in f.files:
            for r in _rows(_load(p), 3, str(p)):
                row = np.array([r[1], 0, 0, 0, 0, r[2]])
                ctx.put(sname, f, num, -1, int(np.rint(r[0] * 1e9)), {"data": row})
    elif k == "odom":
        ctx.add(sname, f, MSG["odom"], {"columns": ODOM_COLS, "frame_id": "odom",
                                        "child_frame_id": ctx.base_frame})
        for num, p in f.files:
            for r in _rows(_load(p), 14, str(p)):
                row = np.r_[r[1:4], reorder_wxyz(r[4:8]), r[8:14]]
                ctx.put(sname, f, num, -1, int(np.rint(r[0] * 1e9)), {"data": row})
    elif k == "pose":
        ctx.add(sname, f, MSG["pose"], {
            "columns": POSE_COLS, "frame_id": "vicon_world",
            "child_frame_id": f.name[len("vicon_"):] if f.name.startswith("vicon_") else f.name})
        for num, p in f.files:
            a = np.asarray(np.load(p, allow_pickle=False), np.float64).reshape(-1)
            if a.shape != (7,):
                raise ConvertError("%s: pose must be (7,) x y z qw qx qy qz, got %s" % (p, a.shape))
            row = np.r_[a[:3], reorder_wxyz(a[3:])]
            ctx.put(sname, f, num, -1, ctx.stamps[num], {"data": row})
    else:  # pragma: no cover - TYPES and this chain are kept in step
        raise ConvertError("no converter for type %r" % k)
    return sname


def _convert_hands(ctx, f, sname):
    first = _load(f.files[0][1]) if f.files else np.zeros((21, 3))
    two = first.ndim == 3
    names = list(HANDS_SUFFIX.get(sname, (sname + "_left", sname + "_right"))) \
        if two else [sname]
    for n in names:
        ctx.add(n, f, MSG["cloud"], {"columns": ["x", "y", "z"],
                                     "frame_id": ctx.hands_frame})
    for num, p in f.files:
        a = np.asarray(_load(p), np.float64)
        if two:
            if a.ndim != 3 or a.shape[0] != 2 or a.shape[2] != 3:
                raise ConvertError("%s: hands must be (2,n,3), got %s" % (p, a.shape))
            parts = [a[0], a[1]]
        else:
            parts = [_rows(a, 3, str(p))]
        for n, d in zip(names, parts):
            ctx.put(n, f, num, -1, ctx.stamps[num], {"data": d}, ("data",))


def refuse_unsafe(v1_dir, out, force):
    v1, o = Path(v1_dir).resolve(), Path(out).resolve()
    if o == v1 or v1 in o.parents:
        raise ConvertError("output %s is inside the source tree %s" % (o, v1))
    if o.exists() and not force:
        raise ConvertError("output %s exists (use --force to overwrite)" % o)
    if not o.parent.is_dir():
        raise ConvertError("output directory %s does not exist" % o.parent)


def convert(v1_dir, out, overrides=None, base_frame="base_link", frame_rate=10.0,
            compression=None, force=False, log=print, hands_frame=HANDS_FRAME):
    """Convert; return ``{"streams": {name: n}, "skipped": [...], "problems": None}``."""
    v1_dir = Path(v1_dir)
    if not v1_dir.is_dir():
        raise ConvertError("source %s is not a directory" % v1_dir)
    refuse_unsafe(v1_dir, out, force)
    folders, skipped = discover(v1_dir, overrides)
    if not folders:
        raise ConvertError("no convertible folders in %s (skipped: %s)" % (v1_dir, skipped))
    for s in skipped:
        log("skipped folder (no mapping; use --map %s=<type>): %s" % (s, s))
    stamps, ts_text = frame_stamps(folders, frame_rate)
    out = Path(out)
    tmp = out.with_name(out.name + ".partial")
    all_ns = [v for v in stamps.values()]
    cap = CaptureFile(tmp, attrs={
        "namespace": "/", "capture_mode": "sampled",
        "sample_rate_hz": float(frame_rate), "timestamps": ts_text,
        "tf_track": [""], "tf_topic": "/tf", "tf_static_topic": "/tf_static",
        "start_recv_ns": min(all_ns) if all_ns else 0,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "converted_from": json.dumps({"source": str(v1_dir.resolve()),
                                      "converter": "cpsl_datasets.convert_v1",
                                      "converter_version": CONVERTER_VERSION}),
        "v1_skipped_folders": skipped,
    }, flush_period_s=1e9)
    ok = False
    try:
        cap.write_tf_static([])
        ctx = _Ctx(cap, base_frame, stamps, compression, hands_frame)
        for f in folders:
            _convert_folder(ctx, f)
        cfg = {n: {"legacy_folder": cap._streams[n].attrs["legacy_folder"]}
               for n in cap.streams()}
        cap.set_root_attr("config_streams", cfg)
        counts = {n: cap.count(n) for n in cap.streams()}
        cap.close()
        ok = True
    finally:
        if not ok:
            try:
                cap._f.close()
            except Exception:
                pass
            tmp.unlink(missing_ok=True)
    # the writer stamps end_recv_ns with conversion time; make it the data's
    import h5py
    with h5py.File(tmp, "r+") as h:
        h.attrs["end_recv_ns"] = max(all_ns) if all_ns else 0
        h.attrs["converted_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    os.replace(tmp, out)
    return {"streams": counts, "skipped": skipped}


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m cpsl_datasets.convert_v1",
        description="Convert a v1 folder-of-frame_N.npy dataset to cpsl_dataset_hdf5 v4 "
                    "(the source is only read).")
    ap.add_argument("v1_dir")
    ap.add_argument("out")
    ap.add_argument("--map", action="append", default=[], metavar="FOLDER=TYPE",
                    help="override the folder->type mapping (types: %s)" % ", ".join(sorted(TYPES)))
    ap.add_argument("--base-frame", default="base_link",
                    help="frame_id of clouds/imu/vel (the v1 recorder's base_frame; "
                         "see hdf5_format.md preset table). default: base_link")
    ap.add_argument("--hands-frame", default=HANDS_FRAME,
                    help="frame_id of hand streams (v1 hand data is raw Leap metres in "
                         "the sensor frame, not base_link). default: %s" % HANDS_FRAME)
    ap.add_argument("--frame-rate", type=float, default=10.0,
                    help="Hz used to synthesize frame stamps when no t column exists")
    ap.add_argument("--compression", choices=["gzip", "lzf"], default=None)
    ap.add_argument("--force", action="store_true", help="overwrite an existing output")
    a = ap.parse_args(argv)
    try:
        res = convert(a.v1_dir, a.out, parse_maps(a.map), a.base_frame, a.frame_rate,
                      a.compression, a.force, hands_frame=a.hands_frame)
    except ConvertError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    for n, c in res["streams"].items():
        print("%-28s %d" % (n, c))
    print("wrote %s (%d streams, %d folders skipped)" % (a.out, len(res["streams"]),
                                                       len(res["skipped"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
