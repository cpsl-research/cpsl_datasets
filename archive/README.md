# archive/

v1 code, kept for reference. Not packaged, not importable as `cpsl_datasets`, not tested.

- `cpsl_ds_v1.py`: the v1 `CpslDS`, which read per-frame `.npy`/image folders.
- `cpsl_plotter_v1.py`: the v1 `CpslPlotter` built on that class (imports `geometries`).
- `dataset_generator_v1.py`: the v1 rclpy recorder that wrote those folders. Superseded by
  the Sensors repo's `dataset_recorder`, which writes HDF5.

They moved here when `CpslDS` was rewritten on HDF5. To read v1 data now, use dsv2-05's
converter to turn the old folders into HDF5, then open the result with the new `CpslDS`.
