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


def test_range_doppler_bins_and_metres(tmp_path):
    ds = CpslDS(build(tmp_path / "v4.h5", "v4"), radar_adc="radar_adc")
    pl = CpslPlotter(ds)
    mag, rng, vel = pl.range_doppler(0)
    cube = ds.get_radar_adc_data(0)
    assert mag.shape == (cube.shape[2], len(rng)) and len(vel) == cube.shape[2]
    # synth metrics carry range_res_m=0.05 but no velocity_res_ms
    assert rng[1] == pytest.approx(0.05)
    assert vel[1] - vel[0] == 1.0
    fig, ax = plt.subplots()
    pl.plot_range_doppler(0, ax=ax)
    assert "range (m)" in ax.get_xlabel() and "Doppler bin" in ax.get_ylabel()
    saved(fig, tmp_path, "rd.png")


def test_range_doppler_empty_raises(tmp_path):
    ds = CpslDS(mixed(tmp_path / "m.h5"), reference="vehicle_odom")
    # odom row 0 precedes the first ADC sample, so the synced cube is empty
    if ds.get_radar_adc_data(0).size:
        pytest.skip("fixture has an ADC sample at odom row 0")
    with pytest.raises(ValueError):
        CpslPlotter(ds).range_doppler(0)


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
