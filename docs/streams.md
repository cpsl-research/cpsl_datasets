# Streams other than radar

Getter, shape, units and frame for each stream; the radar is in [radar.md](radar.md), opening
a file in [index.md](index.md). Snippets assume
`import numpy as np`, `import matplotlib.pyplot as plt` and `from cpsl_datasets import CpslDS`.

## Rules that apply to every stream

- Getters return **what was recorded**, as stored. Nothing is rescaled, rectified or moved to
  another frame unless you ask (`frame=` for lidar and radar point clouds, `get_transform`).
- Two kinds of getter. **Newest-sample** getters (cameras, point clouds, `camera_info`) return
  the newest sample at or before the reference stamp. **Windowed** getters (odometry, velocity,
  IMU, Vector3, Vicon) return every row with a stamp in `(t_ref[idx-1], t_ref[idx]]`, so
  stacking all frames reproduces the stream once, with no gaps or repeats.
- An empty result (no sample yet) is `(0, ...)`-shaped, never `None`, except `get_camera_info`
  (`None`).
- Time is the chosen clock in seconds (the `t` column of windowed getters); recv by default.

## Lidar

Stored: a ragged `PointCloud2` stream with no `vel` column, all columns kept. IcaRAus-style
captures hold `float32` rows of x y z intensity (an `(19968, 4)` cloud on the converted real
file). An Ouster cloud has nine fields: x y z intensity t reflectivity ring ambient range.
**Intensity has no stated unit**: it is stored as the driver published it, and neither this
package nor the Sensors format spec defines it. Treat it as a relative, sensor-specific value.

```python
# synth: getters
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
xyzi = ds.get_lidar_point_cloud_raw(0)            # (n, 4) x y z intensity, sensor frame
cols = ds.get_point_cloud_columns(0, role="lidar", drop_invalid=True)
print(list(cols))                                  # every stored column, 1-D arrays
plt.scatter(cols["x"], cols["y"], s=2)
plt.show()
```

`get_lidar_point_cloud(idx)` is the v1 form: points with `-0.2 < z < 0.1` as `(n, 2)` x y.
`drop_invalid=True` removes rows with non-finite or all-zero xyz. Pass `frame="base_link"` to
`CpslDS` to map x y z through `/tf_static` (v4 files only; otherwise `ValueError`).

**Ouster.** All nine columns come back through `get_point_cloud_columns`; no real Ouster file exists yet.

## Camera and depth

Stored: `Image` streams. The colour camera keeps the message encoding (`rgb8`, `bgr8`, `rgba8`,
`bgra8`, `8UC3`), as `uint8`, `(H, W, C)`. The stream named `camera_depth` is `(H, W)`,
`uint16` (mono16) or `float32` (32FC1). Converted
real files are `bgr8`: `(720, 1280, 3)` and `(480, 640, 3)` for two of them, depth `(480, 848)`.

```python
# synth: human
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
img = ds.get_camera_frame(len(ds) - 1)[..., :3]
if img.size:                              # early frames can be empty
    if ds.camera_encoding.startswith("bgr"):
        img = img[..., ::-1]              # show as RGB
    plt.imshow(img)
    plt.show()
```

Depth is `ds.get_camera_depth_frame(idx)`. Images are never transformed.

## Odometry, velocity and IMU

Stored: `Odometry` rows `x y z qx qy qz qw vx vy vz wx wy wz` (position in metres, quaternion
**x-first**, v1 was w-first; frame `odom`, child `base_link`); a `Twist` stream for velocity
(without one, the odometry twist is used); `Imu` rows `qx qy qz qw wx wy wz ax ay az`.

| Getter | Returns |
|---|---|
| `get_vehicle_odom_data(idx)` | `(m, 14)`: `t`, then the 13 odometry columns |
| `get_vehicle_vel_data(idx)` | `(m, 3)`: `t vx wz` |
| `get_imu_full_data(idx)` | `(m, 7)`: `t wx wy wz ax ay az` |
| `get_imu_orientation_rad(idx)` | yaw in radians, `[-pi, pi]`, from the newest quaternion; `nan` if none |

```python
# synth: v4
from cpsl_datasets import CpslDS
import numpy as np
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
odom = np.vstack([ds.get_vehicle_odom_data(i) for i in range(len(ds))])
plt.plot(odom[:, 1], odom[:, 2])           # columns: t, x, y, ...
plt.show()
```

## Frames and tf

`ds.get_transform(target, source)` returns a `4 x 4` `float64` `T_target_source`; give `idx` or
`t_ns` to evaluate a tracked (moving) edge. It needs a v4 file: v2, v3 and converted v1 files
have no tf and raise `ValueError`. `ds.capture.tf_static()` lists the static edges.
`get_tf_message(idx)` returns the raw `/tf` message as `(n, 7)` x y z qx qy qz qw, without
frame names.

```python
# synth: v4
from cpsl_datasets import CpslDS

ds = CpslDS("capture.h5")
T = ds.get_transform("base_link", "radar_0")   # frame names: inspect
print(T[:3, 3], T.dtype)                    # translation in metres
```

## Vicon

Stored: one `TransformStamped` stream per segment, `x y z` (metres) `qx qy qz qw`, frame
`vicon_world`, child frame per segment (`x500_8` on the real vicon file). `get_vicon_data(idx)`
returns `(m, 7)` with **no time column**, or a dict keyed by stream when several segments are
bound; select one with `name=` (stream or child frame). `wxyz=True` reorders the quaternion to
`qw qx qy qz`. `ds.vicon_frames` maps stream to `(frame_id, child_frame_id)`.

```python
# synth: getters
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
print(ds.vicon_frames)
name = next(iter(ds.vicon_frames))           # a stream name; inspect lists them
v = ds.get_vicon_data(1, name=name)
plt.plot(v[:, 0], v[:, 1], "o-")
plt.show()
```

## Hands (Leap Motion)

> **Hands are raw Leap-sensor coordinates in metres. They are not in the robot base frame,
> and neither `frame=` nor `get_transform` moves them.**

Stored: two `PointCloud2` streams, `hand_left` and `hand_right`, 21 points (`x y z`) per
sample, `frame_id` `cpsl_human_movement/leap_link`. Joint order: palm, then thumb, index,
middle, ring, pinky, four joints each. An absent hand is 21 zero rows.
`get_hand_tracking_data(idx, hand="left")` returns `(21, 3)`; `hand_present(idx, "left")` says
whether the hand was seen. With both hands bound, `hand=None` raises `ValueError`; with an
absent hand the getter returns `zeros((21, 3))` and `hand_present` is `False`.

```python
# synth: human
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
for i in range(len(ds)):
    print(i, ds.hand_present(i, "left"), ds.hand_present(i, "right"))
pts = ds.get_hand_tracking_data(0, hand="right")
plt.scatter(pts[:, 0], pts[:, 1])
plt.show()
```

**v1 hands.** Hands in converted v1 datasets are also raw Leap coordinates, whatever the
`frame_id` says (older converted files were stamped `base_link`, wrongly). The console memo
above found no code that moved v1 hands to the base frame. Sensors [reading_datasets.md](https://github.com/cpsl-research/CPSL_ROS2_Sensors/tree/main/src/dataset_generator/docs)
(line 172) says v1 hands were in the base frame. That conflicts with the memo; treat v1 hands
as raw Leap until someone checks.

## Leap images

Stored: two `mono8` `Image` streams, `leap_left` and `leap_right` (`(512, 512)` `uint8` on the
one real file that has them). Getters are `get_leap_motion_image_frame_left(idx)` and
`..._right(idx)`; both may be empty `(0, 0)` early in a capture.

```python
# synth: human
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5")
img = ds.get_leap_motion_image_frame_left(len(ds) - 1)
if img.size:
    plt.imshow(img, cmap="gray")
    plt.show()
```

## Appendix: every role, getter and return

Roles are `CpslDS` keyword names; give a stream name to pick one, `False` to switch it off.

| Role | Getter | Returns | Stamp rule |
|---|---|---|---|
| radar_adc | `get_radar_adc_data(idx)` | complex64 `(rx, sample, chirp)` | newest; default reference |
| radar_cfg | `get_radar_cfg()` | dict or `None` | static per file |
| chirp_meta | `get_chirp_meta(idx)` | dict of arrays (`{}` if none) | newest |
| radar_pc | `get_radar_point_cloud(idx)` | `(n, 4)` x y z vel | newest |
| lidar | `get_lidar_point_cloud_raw(idx)` | `(n, 4)` x y z intensity | newest |
| lidar | `get_lidar_point_cloud(idx)` | `(n, 2)` x y, ground cut | newest |
| lidar | `get_point_cloud_columns(idx, role, drop_invalid)` | dict column -> 1-D | newest |
| camera | `get_camera_frame(idx)` | `(H, W, C)` uint8 | newest |
| camera_depth | `get_camera_depth_frame(idx)` | `(H, W)` uint16 or float32 | newest |
| camera_info | `get_camera_info(idx=0)` | dict `K R P D width height ...` or `None` | newest |
| vehicle_odom | `get_vehicle_odom_data(idx)` | `(m, 14)` | window |
| vehicle_vel | `get_vehicle_vel_data(idx)` | `(m, 3)` | window |
| imu | `get_imu_full_data(idx)` | `(m, 7)` | window |
| imu | `get_imu_orientation_rad(idx)` | float yaw (rad) | newest |
| vector3 | `get_vector3(idx)` | `(m, 4)` t x y z | window |
| vicon | `get_vicon_data(idx, name, wxyz)` | `(m, 7)` or dict | window |
| hand_left, hand_right | `get_hand_tracking_data(idx, hand)` | `(21, 3)` | newest |
| leap_left, leap_right | `get_leap_motion_image_frame_left/right(idx)` | `(H, W)` uint8 | newest |
| tf_message | `get_tf_message(idx)` | `(n, 7)` | newest, recv only |
| (tf) | `get_transform(target, source, idx, t_ns)` | `4 x 4` float64 | tracked edge at stamp |
| any | `get_stamps(role)` | int64 ns, whole stream | chosen clock |

`ds.sample_index(role, idx)` gives the raw sample index served (`-1`: none yet). Raw stream
access: `ds.capture.streams()[name]` is a `Stream` with `.msg_type`, `.frame_id`, `.columns`,
`.attrs`, `len(s)`, `s.read(i)`, `s.stamps("recv" | "header")` and `s.named(i)`.
