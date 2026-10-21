"""Synthetic v1 datasets (folder-of-``frame_N.npy``), written with numpy + cv2.

Follows the v1 recorder (archive/dataset_generator_v1.py) and the real v1
datasets seen so far: ``frame_10000.npy`` ... per folder, camera as PNG (BGR),
everything else ``.npy``. Never commit the produced trees: tests build them
in tmp_path.

``build_v1(root, ...)`` returns a ``Truth``: per folder, the frame numbers and
the arrays exactly as written, so tests can compare against what the
converter produced.
"""
from pathlib import Path

import cv2
import numpy as np

T0_S = 1_700_000_000.0  # node-clock seconds of frame 0
FRAME_DT = 0.1
ROWS_PER_FRAME = 3      # IMU / odom / vel rows per save tick

# group name -> folders it creates. `build_v1(include=...)` picks groups.
GROUPS = {
    "cube": ["radar_0_adc"],
    "pc": ["radar_0_pc", "radar_1_pc", "radar_combined_pc"],
    "lidar": ["lidar"],
    "camera": ["camera", "camera_npy", "depth"],
    "imu": ["imu_data", "imu_data_full"],
    "imu_full_only": ["imu_data_full"],
    "vehicle": ["vehicle_vel", "vehicle_odom"],
    "vicon": ["vicon_x500_8"],
    "hands": ["hands", "hand_single"],
    "leap": ["leap_images_left", "leap_images_right"],
}
DEFAULT = ("cube", "pc", "lidar", "camera", "imu", "vehicle", "vicon", "hands", "leap")


class Truth:
    def __init__(self, root, numbers):
        self.root = Path(root)
        self.numbers = list(numbers)   # frame numbers, ascending
        self.frames = {}               # folder -> list of arrays (frame order)
        self.ext = {}                  # folder -> "npy" | "png"

    def folders(self):
        return sorted(self.frames)


def _quat_wfirst(rng, m):
    q = rng.normal(size=(m, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _rows_t(k, m=ROWS_PER_FRAME):
    """Node-clock times of the m rows of frame k (strictly increasing overall)."""
    return T0_S + k * FRAME_DT + np.arange(1, m + 1) * (FRAME_DT / (m + 1))


def build_v1(root, start=10000, n=4, include=DEFAULT, extra_folders=(),
             seed=0):
    """Write a v1 tree under ``root`` (created); return its Truth.

    ``extra_folders``: names of folders no pattern knows (each gets (2,) npy).
    """
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    numbers = list(range(start, start + n))
    tr = Truth(root, numbers)
    want = set()
    for g in include:
        want.update(GROUPS[g])
    want.update(extra_folders)

    def put(folder, make, ext="npy"):
        d = root / folder
        d.mkdir(exist_ok=True)
        arrs = []
        for k, num in enumerate(numbers):
            a = make(k)
            arrs.append(a)
            path = d / ("frame_%d.%s" % (num, ext))
            if ext == "npy":
                np.save(path, a)
            else:
                assert cv2.imwrite(str(path), a)
        tr.frames[folder] = arrs
        tr.ext[folder] = ext

    for f in sorted(want):
        if f.endswith("_adc"):
            # complex128 (rx, sample, chirp) with integer values, like the real ones
            def cube(k):
                re = rng.integers(-2000, 2000, size=(4, 6, 5))
                im = rng.integers(-2000, 2000, size=(4, 6, 5))
                return (re + 1j * im).astype(np.complex128)
            put(f, cube)
        elif f.endswith("_pc"):
            # (n, 4) x y z vel; n varies and one frame is empty
            def pc(k):
                rows = 0 if k == 1 else 3 + k + len(f) % 3
                return rng.normal(size=(rows, 4)).astype(np.float32)
            put(f, pc)
        elif f == "lidar":
            put(f, lambda k: rng.normal(size=(7 + k, 4)).astype(np.float32))
        elif f == "camera":
            put(f, lambda k: rng.integers(0, 255, size=(5, 6, 3), dtype=np.uint8),
                "png")
        elif f == "camera_npy":
            put(f, lambda k: rng.integers(0, 255, size=(5, 6, 3), dtype=np.uint8))
        elif f == "depth":
            put(f, lambda k: rng.integers(0, 60000, size=(5, 6)).astype(np.uint16),
                "png")
        elif f == "imu_data":      # (4,) quaternion w x y z
            put(f, lambda k: _quat_wfirst(rng, 1)[0])
        elif f == "imu_data_full":  # (m, 7) t wx wy wz ax ay az
            def full(k):
                a = np.zeros((ROWS_PER_FRAME, 7))
                a[:, 0] = _rows_t(k)
                a[:, 1:] = rng.normal(size=(ROWS_PER_FRAME, 6))
                return a
            put(f, full)
        elif f == "vehicle_vel":   # (m, 3) t vx wz
            def vel(k):
                a = np.zeros((ROWS_PER_FRAME, 3))
                a[:, 0] = _rows_t(k)
                a[:, 1:] = rng.normal(size=(ROWS_PER_FRAME, 2))
                return a
            put(f, vel)
        elif f == "vehicle_odom":  # (m, 14) t x y z qw qx qy qz vx vy vz wx wy wz
            def odom(k):
                a = np.zeros((ROWS_PER_FRAME, 14))
                a[:, 0] = _rows_t(k)
                a[:, 1:4] = rng.normal(size=(ROWS_PER_FRAME, 3))
                a[:, 4:8] = _quat_wfirst(rng, ROWS_PER_FRAME)
                a[:, 8:] = rng.normal(size=(ROWS_PER_FRAME, 6))
                return a
            put(f, odom)
        elif f.startswith("vicon_"):  # (7,) x y z qw qx qy qz
            def vic(k):
                return np.r_[rng.normal(size=3), _quat_wfirst(rng, 1)[0]]
            put(f, vic)
        elif f == "hands":         # (2, n, 3), n varies per frame
            put(f, lambda k: rng.normal(size=(2, 4 + k, 3)))
        elif f == "hand_single":   # (21, 3)
            put(f, lambda k: rng.normal(size=(21, 3)))
        elif f == "leap_images_left":
            put(f, lambda k: rng.integers(0, 255, size=(4, 5), dtype=np.uint8),
                "png")
        elif f == "leap_images_right":
            put(f, lambda k: rng.random(size=(4, 5)).astype(np.float32))
        else:                      # unknown folder
            put(f, lambda k: rng.normal(size=(2,)))
    return tr
