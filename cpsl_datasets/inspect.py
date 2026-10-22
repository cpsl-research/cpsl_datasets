"""Summarise a cpsl_dataset_hdf5 capture.

    python -m cpsl_datasets.inspect FILE [--validate]
"""
import argparse
import sys

import numpy as np

from .hdf5 import Capture, validate


def clock_report(stream):
    """Recv-vs-header clock offset of a stream (stamp_recv_ns - stamp_header_ns).

    Returns a dict: n, n_unstamped, and (when any header stamp exists) mean_ms,
    std_ms, min_ms, max_ms.
    """
    rec, hdr = stream.stamps("recv"), stream.stamps("header")
    ok = hdr != -1
    out = {"n": int(len(rec)), "n_unstamped": int((~ok).sum())}
    if ok.any():
        d = (rec[ok] - hdr[ok]) / 1e6
        out.update(mean_ms=float(d.mean()), std_ms=float(d.std()),
                   min_ms=float(d.min()), max_ms=float(d.max()))
    return out


def _clock_line(r):
    if r["n"] == 0:
        return "no samples"
    if "mean_ms" not in r:
        return "header stamps are -1 (no header): use recv"
    s = "recv-header mean %.3f ms, std %.3f, min %.3f, max %.3f" % (
        r["mean_ms"], r["std_ms"], r["min_ms"], r["max_ms"])
    if r["n_unstamped"]:
        s += " (%d of %d header stamps are -1)" % (r["n_unstamped"], r["n"])
    return s


def fill_ratio(stream, rate_hz, span_ns):
    """Rows of a ``sampled`` stream over the ticks of the capture span
    (``floor(span * rate) + 1``), at most 1.0. ~1 means a new message arrived
    every tick; low means ticks passed with nothing new (a slower sensor, or
    dropped messages). None without a rate or a span."""
    if not rate_hz or span_ns <= 0:
        return None
    ticks = max(int(span_ns / 1e9 * rate_hz) + 1, len(stream))
    return len(stream) / ticks


def _mode_lines(c, streams):
    rate = c.sample_rate_hz
    out = ["capture_mode: %s  sample_rate_hz: %s" % (
        c.capture_mode or "unknown (not recorded)",
        "%g" % rate if rate is not None else "not recorded")]
    allt = [s.stamps("recv") for s in streams.values() if len(s)]
    span = int(max(t.max() for t in allt) - min(t.min() for t in allt)) if allt else 0
    for n, s in streams.items():
        m = s.mode or "unknown"
        line = "  %-18s mode %-8s" % (n, m)
        if m == "sampled":
            f = fill_ratio(s, rate, span)
            line += " fill %.2f" % f if f is not None else " fill n/a (no sample_rate_hz)"
        out.append(line)
    return out


def report(path, out=print):
    """Print the summary of ``path`` through ``out``: format, capture_mode and
    sample_rate_hz, each stream (type, mode, samples, fill ratio, frame_id, time
    span), the clock report, and the radar cfg."""
    with Capture(path) as c:
        out("file: %s" % path)
        out("format: %s  version: %d  closed_cleanly: %s" % (c.format, c.version, c.closed_cleanly))
        for w in c.warnings:
            out("WARNING: %s" % w)
        streams = c.streams()
        out("")
        out("%-18s %-24s %8s  %-16s %s" % ("stream", "msg_type", "samples", "frame_id", "time span"))
        for n, s in streams.items():
            t = s.stamps("recv")
            span = "-" if len(t) == 0 else "%.3f s" % ((t[-1] - t[0]) / 1e9)
            out("%-18s %-24s %8d  %-16s %s" % (n, s.type, len(s), s.frame_id or "-", span))
        out("")
        for line in _mode_lines(c, streams):
            out(line)
        out("")
        out("clock report (recv vs header):")
        for n, s in streams.items():
            out("  %-18s %s" % (n, _clock_line(clock_report(s))))
        out("")
        edges = c.tf_static()
        out("tf_static: %d frames" % len(edges))
        for p, ch, xyz, q in edges:
            out("  %s -> %s  xyz=%s" % (p, ch, np.round(xyz, 4).tolist()))
        pairs = c.tf_pairs()
        out("tracked tf pairs: %d" % len(pairs))
        for (p, ch), g in pairs.items():
            out("  %s -> %s  n=%d" % (p, ch, g["stamp_recv_ns"].shape[0]))


def main(argv=None):
    """Command line entry: ``python -m cpsl_datasets.inspect FILE [--validate]``."""
    ap = argparse.ArgumentParser(prog="python -m cpsl_datasets.inspect", description=__doc__)
    ap.add_argument("file")
    ap.add_argument("--validate", action="store_true", help="also check the file against hdf5_format.md")
    args = ap.parse_args(argv)
    report(args.file)
    if args.validate:
        probs = validate(args.file)
        print("")
        if probs:
            print("validate: %d problem(s)" % len(probs))
            for p in probs:
                print("  - %s" % p)
            return 1
        print("validate: no problems")
    return 0


if __name__ == "__main__":
    sys.exit(main())
