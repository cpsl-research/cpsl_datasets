# Radar data

> **cpsl_datasets returns raw radar data and does not process it.** It gives you the ADC
> cube, the recorded radar config, the chirp metadata and the recorded point cloud, exactly as
> stored. No range FFT, Doppler, CFAR, calibration or beamforming. Signal processing lives in
> [CPSL_TI_Radar](https://github.com/davidmhunt/CPSL_TI_Radar) (which includes `radar_gui`).

Open a file as in [index.md](index.md). Snippets name the cube stream because the synthetic
file has two. Real-file numbers below come from one sensor (`radar06`); nothing here has been
measured on another.

## ADC cube

Stored: an `ADCDataCube` stream. `data` is `int16`, `(N, 2, rx, sample, chirp)`; the `2` is
real and imaginary. `get_radar_adc_data(idx)` returns `complex64` `(rx, sample, chirp)`, i.e.
`d[0] + 1j * d[1]` in raw ADC counts, no scaling. Cube sizes differ per file (the
`radar06` cube is `(1, 3300, 255)`; two converted files are `(4, 70, 120)` and `(4, 63, 300)`).
Read the group's `axes` attribute to see the stored axis order instead of assuming it.

```mermaid
flowchart LR
    D["stored data<br/>int16 (N, 2, rx, sample, chirp)"] -->|"frame idx"| C["(2, rx, sample, chirp)<br/>read(idx, raw=True)"]
    C -->|"d[0] + 1j*d[1]"| G["complex64 (rx, sample, chirp)<br/>get_radar_adc_data(idx)"]
    G --> S1["cube[0, :, 5]<br/>one chirp: (sample,)"]
    G --> S2["cube[:, :, :16]<br/>first 16 chirps"]
```

*Figure 2. Cube axes and slicing.*

```python
# synth: v4
from cpsl_datasets import CpslDS
import numpy as np
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5", radar_adc="radar_adc", vehicle_vel="twist_st")
cube = ds.get_radar_adc_data(0)                          # (rx, sample, chirp) complex64
raw = ds.streams["radar_adc"].read(0, raw=True)          # (2, rx, sample, chirp) int16
assert np.allclose(raw[0] + 1j * raw[1], cube)
chirp = cube[0, :, 0]                                    # one chirp, receiver 0
plt.plot(chirp.real, label="real")
plt.plot(chirp.imag, label="imag")
plt.legend()
plt.show()
```

## Radar config and chirp parameters

`get_radar_cfg()` returns the config the radar ran with (v3 and later), or `None` if there is
none: v2 files, converted v1 files, or a recorded status other than `"ok"`. It is a dict with
keys `changed, metrics, path, sha256, status, text, topic`; `text` is the `.cfg` text and
`metrics` the decoded chirp numbers (`n_rx`, `num_samples`, `n_chirps`, `range_res_m`,
`frame_rate_hz`, `bandwidth_mhz`, ...; full list in the appendix).

```python
# synth: v4
from cpsl_datasets import CpslDS

ds = CpslDS("capture.h5", radar_adc="radar_adc", vehicle_vel="twist_st")
cfg = ds.get_radar_cfg()
if cfg is not None:
    m = cfg["metrics"]
    print(m["range_res_m"], m["frame_rate_hz"])
    cube = ds.get_radar_adc_data(0)
    print((m["n_rx"], m["num_samples"], m["n_chirps"]) == cube.shape)   # cube matches cfg
```

On `radar06`, `metrics` gives 1 receiver, 3300 samples, 255 chirps, a 510.3 ms frame period,
3499.5 MHz bandwidth, 0.0428 m range resolution and TDM scheme, and the cube shape equals
`(n_rx, num_samples, n_chirps)`.

## Chirp metadata

Newer recorders store an `adc_meta` stream (`ADCChirpMeta`) beside the cube, one sample per
cube. `get_chirp_meta(idx)` returns a dict of the stored arrays: per-chirp arrays of length
`n_chirps` (`record_valid`, `adc_complete`, `has_time`, `time_interpolated`, `flags`, `run_idx`,
`global_chirp_idx`, `ts_ticks`, `sat_slices`, `sat_ref_lag`, `sat_slices_aligned`, `t_s`),
per-frame scalars (`frame_index`, `run_known`, `run_idx_frame`, `records_valid`,
`records_invalid`, `records_other_run`) and `prev_tail_sat`. `t_s` is the radar's own clock in
seconds, not epoch time. `{}` when there is none yet.

```python
# synth: v4
from cpsl_datasets import CpslDS

ds = CpslDS("capture.h5", radar_adc="radar_adc", vehicle_vel="twist_st")
meta = ds.get_chirp_meta(0)
print(meta["record_valid"].all(), meta["t_s"][:3])
```

## Point cloud

Stored: a ragged `PointCloud2` stream with a `vel` column. `get_radar_point_cloud(idx)` returns
`(n, 4)` x y z vel (metres, m/s), `float32` on real files; empty `(0, K)` when there are no
detections. Points are in the sensor frame (`frame_id`) unless you open the file with
`frame="base_link"`, which maps x y z through `/tf_static` (v4 files only). With several
radars, name one: `radar_pc="radar_combined_pc"`, or set `CPSL_STREAMS` as in the README.

```python
# synth: v4
from cpsl_datasets import CpslDS
import matplotlib.pyplot as plt

ds = CpslDS("capture.h5", radar_adc="radar_adc", vehicle_vel="twist_st")
pc = ds.get_radar_point_cloud(0)
if len(pc):
    plt.scatter(pc[:, 0], pc[:, 1], c=pc[:, 3])
    plt.colorbar(label="vel (m/s)")
    plt.show()
```

## Timing and syncing

Every sample has two stamps. `stamp_recv_ns` is the recording host's clock when the message
arrived; it is shared by all streams. `stamp_header_ns` is the stamp inside the message, set by
the driver; `-1` means none. **The default clock is recv**, so all streams align on one clock.

On `radar06` the radar's recv stamp is later than its header stamp by a large, jittery amount
(`recv - header`, from `inspect.clock_report` on the file's `radar_adc` stream):

| Stream (`radar06`, 66 frames) | mean | std | min | max |
|---|---|---|---|---|
| `radar_adc` | 112.1 ms | 4.9 ms | 104.8 ms | 126.3 ms |
| `adc_meta` | 104.4 ms | 1.5 ms | 102.7 ms | 108.4 ms |

This is one sensor on one capture; it has not been repeated on other radars. With `clock="header"` the radar frames therefore line up ~100 ms earlier against recv-stamped
streams. For one query time on `radar06` the two clocks picked adjacent samples (19 versus 20).

```python
# synth: v4
from cpsl_datasets import CpslDS
from cpsl_datasets.inspect import clock_report

ds = CpslDS("capture.h5", radar_adc="radar_adc", vehicle_vel="twist_st")
print(clock_report(ds.capture.streams()["radar_adc"]))   # n, n_unstamped, mean_ms, std_ms, ...
print(ds.get_stamps("radar_adc")[:3])                      # int64 ns, chosen clock
```

![Timing: stamp axes, radar offset and odometry window](figures/radar_timing.png)

*Figure 3. Schematic: header and recv stamps of a radar, an odometry stream on recv stamps,
the reference stamp and the odometry window. Spacing is illustrative; the ~112 ms offset is
`radar06`.*

**Frame-synced getters.** Frame `idx` is the idx-th radar cube. Cube, point cloud, camera and
lidar getters return the newest sample at or before that cube's stamp (ties go to the later
row); odometry, velocity, IMU, Vector3 and Vicon return the window
`(t_ref[idx-1], t_ref[idx]]` (for `idx` 0, everything up to `t_ref[0]`). `ds.sample_index(role,
idx)` tells which raw sample was served.

**Header clock.** `clock="header"` raises `ValueError` if any bound stream has header stamps of
`-1`. That is the case for every converted v1 file (`inspect` says "header stamps are -1"),
and `/tf` messages have no header stamp, so also pass `tf_message=False`.

```python
# synth: v4
from cpsl_datasets import CpslDS

ds = CpslDS("capture.h5", radar_adc="radar_adc", clock="header", tf_message=False,
            vehicle_vel=False)
print(len(ds), ds.get_radar_adc_data(0).shape)
```

## Appendix: radar `metrics` keys

`adc_start_us, azimuth_res_deg, bandwidth_mhz, bytes_per_chirp, bytes_per_frame,
bytes_per_sample, calculator, calculator_version, center_ghz, chirp_us, chirps_per_loop,
doppler_bins, doppler_step_ms, frame_period_ms, frame_rate_hz, idle_us, lambda_mm,
loop_period_us, lvds_data_fmt, max_range_ideal_m, max_range_m, max_velocity_ms, n_az_virtual,
n_chirps, n_loops, n_rx, n_tx, n_virtual, num_frames, num_samples, ramp_us, range_res_m,
sample_rate_ksps, sampling_us, scheme, slope_mhz_us, start_ghz, velocity_res_ms`. Listed from
`radar06`; the synthetic test file has only a few. The `get_radar_cfg()["text"]` is the
authoritative `.cfg`.
