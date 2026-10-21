import os
import importlib.util
import subprocess
import sys

import h5py
import numpy as np
import pytest

from cpsl_datasets.hdf5 import Capture, validate
from cpsl_datasets.inspect import clock_report, main

from synth_h5 import RECV_OFFSET_NS, build


def test_clock_report(tmp_path):
    p = build(tmp_path / "a.h5", "v4")
    with Capture(p) as c:
        r = clock_report(c["imu" if "imu" in c.streams() else "odom"])
        assert 5.0 <= r["mean_ms"] <= 6.0 and r["std_ms"] < 1.0
        assert r["min_ms"] >= RECV_OFFSET_NS / 1e6 and r["n_unstamped"] == 0
        r = clock_report(c["cmd_vel"])
        assert r["n_unstamped"] == 3 and "mean_ms" not in r
        assert clock_report(c["radar_adc_empty"])["n"] == 0


def test_cli_output(tmp_path, capsys):
    p = build(tmp_path / "a.h5", "v4")
    assert main([str(p), "--validate"]) == 0
    out = capsys.readouterr().out
    assert "version: 4" in out and "closed_cleanly: True" in out
    assert "radar_adc_empty" in out and "ADCDataCube" in out
    assert "recv-header mean" in out
    assert "cmd_vel" in out and "header stamps are -1" in out
    assert "base_link -> radar" in out and "odom -> base_link  n=5" in out
    assert "validate: no problems" in out


def test_cli_reports_problems_and_unclosed(tmp_path, capsys):
    p = build(tmp_path / "u.h5", "unclosed")
    with h5py.File(p, "r+") as f:
        f["imu"].attrs["n_samples"] = 9 if "imu" in f else None
    with pytest.warns(UserWarning):
        rc = main([str(p), "--validate"])
    out = capsys.readouterr().out
    assert rc == 1 and "WARNING: closed_cleanly is False" in out and "n_samples=9" in out


def test_module_runs(tmp_path):
    p = build(tmp_path / "a.h5", "v2")
    r = subprocess.run([sys.executable, "-m", "cpsl_datasets.inspect", str(p)],
                       capture_output=True, text=True)
    assert r.returncode == 0 and "radar_pc" in r.stdout


WRITER = os.environ.get("CPSL_HDF5_WRITER")


@pytest.mark.skipif(not WRITER, reason="set CPSL_HDF5_WRITER to Sensors' hdf5_writer.py")
def test_parity_with_real_writer(tmp_path):
    spec = importlib.util.spec_from_file_location("hdf5_writer_real", WRITER)
    w = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(w)
    out = tmp_path / "real.h5"
    cf = w.CaptureFile(out, attrs={"namespace": "/", "tf_track": ["odom->base_link"],
                                   "config_streams": {}, "tf_topic": "/tf",
                                   "tf_static_topic": "/tf_static"})
    cf.add_stream("radar_adc", {"msg_type": "raw_radar_msgs/msg/ADCDataCube",
                                "columns": ["real", "imag"], "frame_id": "radar",
                                "axes": ["re_im", "rx", "sample", "chirp"]})
    cf.add_stream("radar_pc", {"msg_type": "sensor_msgs/msg/PointCloud2",
                               "columns": ["x", "y", "z", "vel"], "frame_id": "radar"})
    cf.add_stream("odom", {"msg_type": "nav_msgs/msg/Odometry", "frame_id": "odom",
                           "child_frame_id": "base_link",
                           "columns": "x y z qx qy qz qw vx vy vz wx wy wz".split()})
    cf.add_stream("cmd_vel", {"msg_type": "geometry_msgs/msg/Twist",
                              "columns": "vx vy vz wx wy wz".split()})
    cf.add_stream("silent", {"msg_type": "geometry_msgs/msg/Twist",
                             "columns": "vx vy vz wx wy wz".split()})
    rng = np.random.default_rng(1)
    for i in range(4):
        t = 10**18 + i * 10**8
        cf.append("radar_adc", t, t + 5, {"data": rng.integers(-9, 9, (2, 3, 8, 2)).astype(np.int16)})
        cf.append("radar_pc", t, t + 5,
                  {"data": rng.normal(size=(i % 3, 4)).astype(np.float32)}, ragged=("data",))
        cf.append("odom", t, t + 5, {"data": rng.normal(size=13)})
        cf.append("cmd_vel", -1, t + 5, {"data": rng.normal(size=6)})
    cf.write_tf_static([("base_link", "radar", [0, 0, 1], [0, 0, 0, 1], 1, 2)])
    cf.close()
    assert validate(out) == []
    with Capture(out) as c:
        assert c.closed_cleanly and c.version == 4
        assert len(c["radar_adc"]) == 4 and c["radar_adc"][0].shape == (3, 8, 2)
        assert [len(c["radar_pc"][i]) for i in range(4)] == [0, 1, 2, 0]
        assert len(c["silent"]) == 0
        assert c.static_chain("radar", "base_link")[2, 3] == 1.0
