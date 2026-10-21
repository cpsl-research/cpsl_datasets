"""Transform helpers for format v4 captures (numpy only, no ROS).

Every transform is T_parent_child (maps child coordinates into parent).
Follows reading_datasets.md section 4. ``f`` is an open h5py File (or
anything with ``f['tf_static']``), ``g`` a tracked ``/tf/<p>__<c>`` group.
"""
import numpy as np


def quat_to_T(xyz, q):
    """4x4 homogeneous matrix from x y z and a quaternion (qx qy qz qw)."""
    q = np.asarray(q, np.float64)
    x, y, z, w = q / np.linalg.norm(q)
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = xyz
    return M


def static_edges(f):
    """List of (parent, child, xyz, quat_xyzw) from /tf_static ([] if absent)."""
    if "tf_static" not in f:
        return []
    g = f["tf_static"]
    parent = [str(s) for s in g["parent"].asstr()[:]]
    child = [str(s) for s in g["child"].asstr()[:]]
    xyz, quat = g["xyz"][:], g["quat_xyzw"][:]
    return [(parent[i], child[i], xyz[i], quat[i]) for i in range(len(parent))]


def static_chain(f, frame, up_to):
    """T_up_to_frame composed from /tf_static.

    Raises ValueError when there is no static path from ``frame`` up to
    ``up_to``.
    """
    edges = {c: (p, x, q) for p, c, x, q in static_edges(f)}
    M = np.eye(4)
    seen = set()
    while frame != up_to:
        if frame not in edges or frame in seen:
            raise ValueError("no static path from %r up to %r" % (frame, up_to))
        seen.add(frame)
        parent, xyz, quat = edges[frame]
        M = quat_to_T(xyz, quat) @ M
        frame = parent
    return M


def pose_at(g, t_ns, interpolate=True):
    """T_parent_child of a tracked tf group at ``t_ns`` (clamped to its range).

    Interpolation is linear in position and nlerp in orientation;
    ``interpolate=False`` returns the nearest sample.
    """
    ts, d = g["stamp_header_ns"][:], g["data"][:]
    if len(ts) == 0:
        raise ValueError("tracked tf group has no samples")
    j = int(np.clip(np.searchsorted(ts, t_ns), 1, max(len(ts) - 1, 1)))
    a, b = max(j - 1, 0), min(j, len(ts) - 1)
    if not interpolate:
        a = a if abs(t_ns - ts[a]) <= abs(ts[b] - t_ns) else b
        return quat_to_T(d[a, :3], d[a, 3:])
    u = 0.0 if ts[b] == ts[a] else float(np.clip((t_ns - ts[a]) / (ts[b] - ts[a]), 0, 1))
    qa, qb = d[a, 3:], d[b, 3:]
    qb = -qb if qa @ qb < 0 else qb
    return quat_to_T((1 - u) * d[a, :3] + u * d[b, :3], (1 - u) * qa + u * qb)
