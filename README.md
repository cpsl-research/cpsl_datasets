# CPSL Datasets

## What changed in 2.0
- HDF5 (`cpsl_dataset_hdf5`, `.h5`) replaces the folder-of-`.npy` dataset format.
- A v1 `.npy` -> HDF5 converter brings old datasets to the new format.
- Poetry is replaced by uv for environments and dependencies.

Package for reading CPSL datasets. A dataset is one `cpsl_dataset_hdf5` file (`.h5`) written
by the Sensors repo's `dataset_recorder`. `CpslDS` opens it and returns frames with the v1
getter names. Old v1 folder datasets are converted to HDF5 first (see below).

Format and reading semantics are defined in the Sensors repo:
[hdf5_format.md](https://github.com/cpsl-research/CPSL_ROS2_Sensors/blob/main/src/dataset_generator/docs/hdf5_format.md),
[reading_datasets.md](https://github.com/cpsl-research/CPSL_ROS2_Sensors/blob/main/src/dataset_generator/docs/reading_datasets.md).

## Documentation

- [User guide](docs/index.md): what a dataset is, `inspect`, opening a file
- [Streams](docs/streams.md): lidar, camera, odometry, IMU, Vicon, hands, Leap images
- [Radar](docs/radar.md): raw ADC cube, point cloud, config, chirp metadata, timing

## Installation
Requires Python 3.10-3.12 and [uv](https://docs.astral.sh/uv/getting-started/installation/).

```
git clone https://github.com/cpsl-research/cpsl_datasets.git
cd cpsl_datasets
git submodule update --init
uv sync                              # core package + dev tools (pytest, nbclient)
uv sync --extra notebooks            # also jupyter, for the viewer notebook
uv run pytest -q                     # tests
```

If a ROS 2 environment is sourced, its `PYTHONPATH` can shadow the locked packages; run
`PYTHONPATH= uv run pytest -q` instead. Every command below is run with `uv run`.
(`--extra athena` installs the `geometries` submodule, needed only for `athena_uav_ds`.)

## Reading an HDF5 capture

```python
import matplotlib.pyplot as plt
from cpsl_datasets import CpslDS

ds = CpslDS("capture.h5")            # roles are auto-detected from message types
print(ds)                            # file, frames, bound roles
print(list(ds.capture.streams()))    # every stream in the file
print(ds.roles)                      # role -> stream name that was bound

cloud = ds.get_radar_point_cloud(0)  # (n, 4) x y z vel
plt.scatter(cloud[:, 0], cloud[:, 1])
plt.show()
ds.close()                           # or: with CpslDS(path) as ds:
```

`idx` is a sample of the reference stream (the first non-empty of radar_adc, radar_pc, lidar,
camera; override with `reference="lidar"`). Every other getter returns that stream's newest
sample at or before the reference sample. `num_frames` is the reference length. The full
constructor, every getter and its return shape are in the `CpslDS` docstring
(`help(CpslDS)`). Getters: `get_radar_adc_data`, `get_radar_cfg`, `get_radar_point_cloud`,
`get_lidar_point_cloud(_raw)`, `get_camera_frame`, `get_camera_depth_frame`,
`get_vehicle_odom_data`, `get_vehicle_vel_data`, `get_imu_full_data`, `get_vector3`,
`get_vicon_data`, `get_hand_tracking_data`, `get_leap_motion_image_frame_left/right`,
`get_stamps`, `get_transform`.

Choosing streams. A capture with several radars or lidars is ambiguous and raises
`ValueError` at construction; name the stream, or switch a role off with `False`:

```python
ds = CpslDS("multi_radar.h5", radar_pc="radar_pc_1", lidar=False)
```

Clock and frame:

- `clock="recv"` (default) aligns on `stamp_recv_ns`, the recording host's clock, shared by
  every stream. `clock="header"` is opt-in and only meaningful when the drivers share a clock.
  On real captures the radar header trails recv by roughly 100 ms, so the two choices pick
  different samples; check the clock report from `inspect` (below).
- `frame=None` (default): point clouds come back in their stored sensor frame (`frame_id`
  attr). `frame="base_link"` maps radar_pc and lidar x, y, z into that frame using
  `/tf_static` (format v4 only; otherwise `ValueError`). ADC cubes and images are never
  transformed.
- Hands are raw Leap-sensor metres in the `leap_link` frame, never transformed.

**No radar processing.** `cpsl_datasets` returns the raw ADC cube and the recorded radar cfg
as stored and does nothing to them: no range-FFT, Doppler, CFAR or calibration. For radar
signal processing use radar_gui or CPSL_TI_Radar.

## Converting a v1 dataset

A v1 dataset is a folder of per-sensor subfolders of `frame_N.npy` / `.png`. Convert it once
to HDF5, then read the result as above:

```
uv run python -m cpsl_datasets.convert_v1 /data/v1_dataset out.h5 \
    --base-frame base_link --frame-rate 10
```

- The v1 folder is only ever read, never modified. An existing `out.h5` is not overwritten
  without `--force`.
- Stream names follow the Sensors recorder (`radar_0_adc` -> `radar_adc`, `depth` ->
  `camera_depth`, `leap_images_left` -> `leap_left`, ...). Folders no name pattern knows are
  printed and listed in the root attr `v1_skipped_folders`.
- `--map folder=type` forces the type of a folder whose name is not recognised
  (`--map my_lidar=lidar`; `folder=skip` drops it). Repeatable.
- `--base-frame` is the `frame_id` stamped on clouds, IMU and velocity (default `base_link`).
  `--hands-frame` is the one for hand streams (default `cpsl_human_movement/leap_link`, since
  v1 hands are raw Leap metres, not base frame).
- v1 stored no header stamps. Row streams take their own `t`; per-frame streams take it from
  the odometry/vel/IMU rows when present, else `k / --frame-rate`. The method is recorded in
  the root attr `timestamps`.
- `--compression gzip|lzf` compresses the output.

## Inspecting a file

```
uv run python -m cpsl_datasets.inspect capture.h5 [--validate]
```

Prints the streams (type, samples, frame_id, time span) and a clock report: the recv minus
header offset per stream, or that a stream has no header stamps. `--validate` also checks the
file against `hdf5_format.md` and prints each problem, or `validate: no problems`.

## API reference

A generated reference (every `CpslDS` kwarg with its default, all getters, `Capture`,
`Stream`, `inspect`, `convert_v1`) is built from the docstrings with
[pdoc](https://pdoc.dev) (a dev dependency); the output in `docs/api/` is not committed:

```
uv run pdoc --docformat numpy -o docs/api cpsl_datasets.cpsl_ds cpsl_datasets.hdf5 \
    cpsl_datasets.inspect cpsl_datasets.convert_v1
```

Open `docs/api/index.html`.

## Viewer notebook

`notebooks/dataset_viewer.ipynb` runs `inspect`, opens the file with `CpslDS` and plots
every stream it has, one cell per role; a cell for an absent stream prints why and skips.
Install with `uv sync --extra notebooks`, then put the capture in `.env` at the repo root:

```
CPSL_DATASET_PATH=/data/captures/capture.h5
# only where auto-detect is ambiguous (several radars): JSON role -> stream name
CPSL_STREAMS={"radar_pc": "radar_pc_1"}
```

and run `uv run jupyter lab notebooks/dataset_viewer.ipynb`. The radar ADC cell shows the
magnitude of the raw cube; it does no radar processing.

## v1 datasets and code

The previous folder-based `CpslDS`, its plotter and the v1 recorder are kept in `archive/`
for reference (not packaged, not tested). The old API as a whole is on the `v1` branch (tag
`v1.0`, commit cfdd785), before the HDF5 rewrite. To use old data with the current API, convert it as above.

## Coordinate frames

Data is stored in the ROS 2 FLU convention (+X forward, +Y left, +Z up). Odometry
quaternions are x-first (`qx qy qz qw`) as stored; v1 was w-first.
