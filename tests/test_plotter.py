"""CpslPlotter smoke tests on synthetic captures (Agg backend)."""
import ast
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from cpsl_datasets import CpslDS
from cpsl_datasets.plotters import cpsl_plotter
from cpsl_datasets.plotters.cpsl_plotter import CpslPlotter

from synth_h5 import build
from test_cpsl_ds import mixed


@pytest.fixture(autouse=True)
def _close():
    yield
    plt.close("all")


def saved(fig, tmp_path, name):
    p = tmp_path / name
    fig.savefig(p)
    assert p.stat().st_size > 0
    return p


def test_no_geometries_import():
    tree = ast.parse(pathlib.Path(cpsl_plotter.__file__).read_text())
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            mods.add(n.module or "")
        elif isinstance(n, ast.Import):
            mods |= {a.name for a in n.names}
    assert not any("geometries" in m for m in mods)


def test_no_range_doppler():
    assert not hasattr(CpslPlotter, "range_doppler")
    assert not hasattr(CpslPlotter, "plot_range_doppler")


def test_odom_trajectory(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    fig, ax = plt.subplots()
    CpslPlotter(ds).plot_odom_trajectory(ax=ax)
    x, y = ax.lines[0].get_data()
    assert list(x) == list(range(20)) and "20 samples" in ax.get_title()
    saved(fig, tmp_path, "odom.png")


def test_odom_trajectory_absent_raises(tmp_path):
    ds = CpslDS(build(tmp_path / "v4.h5", "v4"), radar_adc="radar_adc", vehicle_odom=False)
    with pytest.raises(ValueError):
        CpslPlotter(ds).plot_odom_trajectory()


def test_tf_trajectory(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    fig, ax = plt.subplots()
    CpslPlotter(ds).plot_tf_trajectory("base_link", "lidar", ax=ax)
    x, y = ax.lines[0].get_data()
    assert len(x) == ds.num_frames and np.allclose(x, 1.0)
    saved(fig, tmp_path, "tf.png")


def test_lidar_and_camera_bgr(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    pl = CpslPlotter(ds)
    fig, axs = plt.subplots(1, 2)
    pl.plot_2d_lidar_detections(0, ax=axs[0])
    pl.plot_camera_data(0, ax=axs[1])
    assert ds.camera_encoding == "bgr8"
    shown = np.asarray(axs[1].images[0].get_array())
    raw = ds.get_camera_frame(0)
    assert np.array_equal(shown, raw[..., ::-1])  # bgr8 -> rgb for display
    saved(fig, tmp_path, "lc.png")


def test_compilation_absent_roles(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))  # no radar_pc
    fig, axs = plt.subplots(1, 3)
    CpslPlotter(ds).plot_compilation(idx=0, axs=axs)
    assert [t.get_text() for t in axs[0].texts] == ["absent"]
    saved(fig, tmp_path, "comp.png")


def test_compilation_v4_fixture(tmp_path):
    ds = CpslDS(build(tmp_path / "v4.h5", "v4"), radar_adc="radar_adc")
    fig, axs = plt.subplots(1, 3)
    CpslPlotter(ds).plot_compilation(idx=0, axs=axs)
    saved(fig, tmp_path, "comp2.png")


def _getters(tmp_path):
    from synth_h5 import build_getters
    return CpslDS(build_getters(tmp_path / "g.h5"))


def test_plot_imu(tmp_path):
    ds = _getters(tmp_path)
    fig, axs = plt.subplots(2, 1)
    CpslPlotter(ds).plot_imu(ax=axs)
    assert len(axs[0].lines) == 3 and len(axs[1].lines) == 3
    assert len(axs[0].lines[0].get_xdata()) == 20
    saved(fig, tmp_path, "imu.png")


def test_plot_vehicle_vel(tmp_path):
    ds = _getters(tmp_path)
    fig, ax = plt.subplots()
    CpslPlotter(ds).plot_vehicle_vel(ax=ax)
    assert len(ax.lines) == 2 and "20 samples" in ax.get_title()
    saved(fig, tmp_path, "vel.png")


def test_plot_imu_absent_raises(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    with pytest.raises(ValueError):
        CpslPlotter(ds).plot_imu()


def test_plot_hands_3d_and_absent(tmp_path):
    from synth_h5 import build_human_movement
    ds = CpslDS(build_human_movement(tmp_path / "h.h5"))
    pl = CpslPlotter(ds)
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    pl.plot_hands_3d(1, ax=ax)  # both hands present
    assert len(ax.collections) == 2
    assert [t.get_text() for t in ax.get_legend().get_texts()] == ["left", "right"]
    assert "leap_link" in ax.get_xlabel()
    saved(fig, tmp_path, "h1.png")
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(projection="3d")
    pl.plot_hands_3d(2, ax=ax2)  # left hand absent in sample 2
    assert "left (absent)" in [t.get_text() for t in ax2.get_legend().get_texts()]
    saved(fig2, tmp_path, "h2.png")


def test_plot_hands_no_stream_raises(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    with pytest.raises(ValueError):
        CpslPlotter(ds).plot_hands_3d(0)


def test_plot_leap_images(tmp_path):
    from synth_h5 import build_human_movement
    ds = CpslDS(build_human_movement(tmp_path / "h.h5"))
    fig, axs = plt.subplots(1, 2)
    CpslPlotter(ds).plot_leap_images(2, axs=axs)
    assert all(len(a.images) == 1 for a in axs)
    assert np.array_equal(np.asarray(axs[0].images[0].get_array()),
                          ds.get_leap_motion_image_frame_left(2))
    fig0, axs0 = plt.subplots(1, 2)  # sample 0: right camera has no frame yet
    CpslPlotter(ds).plot_leap_images(0, axs=axs0)
    assert axs0[1].texts[0].get_text() == "no frame"
    saved(fig, tmp_path, "leap.png")


def test_plot_leap_images_absent(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"))
    fig, axs = plt.subplots(1, 2)
    CpslPlotter(ds).plot_leap_images(0, axs=axs)
    assert [a.texts[0].get_text() for a in axs] == ["absent", "absent"]
