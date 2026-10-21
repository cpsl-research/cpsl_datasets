"""Plots for a ``CpslDS`` (HDF5 capture). numpy + matplotlib only.

Clouds are drawn x (horizontal) against y (vertical) in whatever frame the
dataset returns them (the sensor frame by default, see ``CpslDS`` ``frame=``).
v1 additionally rotated clouds by 90 degrees for display; that is not done here.
Radar signal processing (range-Doppler, etc.) is not done here or in ``CpslDS``;
see radar_gui / CPSL_TI_Radar.
"""
import matplotlib.pyplot as plt
import numpy as np


class CpslPlotter:
    def __init__(self, dataset) -> None:
        self.font_size_axis_labels = 12
        self.font_size_title = 15
        self.font_size_ticks = 12
        self.font_size_legend = 12
        self.plot_x_max = 20
        self.plot_y_max = 20
        self.marker_size = 10
        self.dataset = dataset

    # --- helpers ---------------------------------------------------------
    @staticmethod
    def _ax(ax):
        return plt.subplots()[1] if ax is None else ax

    @staticmethod
    def _finish(show):
        if show:
            plt.show()

    def _plot_cloud(self, pts, ax, title, label, color):
        ax.scatter(pts[:, 0], pts[:, 1], label=label, marker="D", color=color,
                   s=self.marker_size)
        ax.set_title("%s: %d" % (title, pts.shape[0]), fontsize=self.font_size_title)
        ax.set_xlim(-self.plot_x_max, self.plot_x_max)
        ax.set_ylim(-self.plot_y_max, self.plot_y_max)
        ax.set_xlabel("X", fontsize=self.font_size_axis_labels)
        ax.set_ylabel("Y", fontsize=self.font_size_axis_labels)
        ax.tick_params(labelsize=self.font_size_ticks)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.legend(loc="lower right", fontsize=self.font_size_legend)

    # --- point clouds ----------------------------------------------------
    def plot_radar_detections(self, sample_idx, ax=None, show=False):
        ax = self._ax(ax)
        pts = np.asarray(self.dataset.get_radar_point_cloud(idx=sample_idx))
        self._plot_cloud(pts.reshape(-1, max(pts.shape[-1], 2)), ax,
                         "Radar detections", "detections", "red")
        self._finish(show)

    def plot_2d_lidar_detections(self, sample_idx, ax=None, show=False):
        ax = self._ax(ax)
        pts = np.asarray(self.dataset.get_lidar_point_cloud(idx=sample_idx))
        self._plot_cloud(pts.reshape(-1, max(pts.shape[-1], 2)), ax,
                         "2D lidar", "lidar", "blue")
        self._finish(show)

    # v1 name
    plot_2d_lidar_detections_velodyne = plot_2d_lidar_detections

    # --- camera ----------------------------------------------------------
    def plot_camera_data(self, sample_idx, ax=None, show=False):
        """Show the camera frame, honouring ``dataset.camera_encoding``
        (bgr*/ rgb*, mono8, 16-bit and float depth are handled)."""
        ax = self._ax(ax)
        img = np.asarray(self.dataset.get_camera_frame(idx=sample_idx))
        enc = (self.dataset.camera_encoding or "").lower()
        ax.set_title("Camera View")
        if img.size == 0:
            ax.text(0.5, 0.5, "no frame", ha="center", va="center",
                    transform=ax.transAxes)
            self._finish(show)
            return
        if img.ndim == 3 and enc.startswith("bgr"):
            img = img[..., [2, 1, 0] + ([3] if img.shape[2] == 4 else [])]
        if img.ndim == 3 and img.shape[2] == 4:
            img = img[..., :3]
        if img.ndim == 3 and img.shape[2] == 1:
            img = img[..., 0]
        if img.ndim == 2:
            ax.imshow(img, cmap="gray" if enc.startswith(("mono", "8uc1")) else "viridis")
        else:
            ax.imshow(img)
        self._finish(show)

    # --- trajectories ----------------------------------------------------
    def _odom_xy(self):
        s = self.dataset.streams.get("vehicle_odom")
        if s is None:
            raise ValueError("dataset has no vehicle_odom stream")
        data = np.asarray(s._g["data"][:])
        return data[:, 0], data[:, 1]

    def plot_odom_trajectory(self, ax=None, show=False):
        """x-y path of every stored odometry row (the whole stream, not one frame)."""
        ax = self._ax(ax)
        x, y = self._odom_xy()
        if len(x):
            ax.plot(x, y, "-", color="tab:blue", label="odom")
            ax.plot(x[0], y[0], "go", label="start")
            ax.plot(x[-1], y[-1], "rs", label="end")
            ax.legend(fontsize=self.font_size_legend)
        ax.set_title("Odometry trajectory: %d samples" % len(x), fontsize=self.font_size_title)
        ax.set_xlabel("x (m)", fontsize=self.font_size_axis_labels)
        ax.set_ylabel("y (m)", fontsize=self.font_size_axis_labels)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True)
        self._finish(show)

    def plot_tf_trajectory(self, target, source, ax=None, show=False):
        """x-y of ``source`` in ``target`` at each reference sample (tf, v4 files)."""
        ax = self._ax(ax)
        n = self.dataset.num_frames
        xy = np.array([self.dataset.get_transform(target, source, idx=i)[:2, 3]
                       for i in range(n)]).reshape(-1, 2)
        if n:
            ax.plot(xy[:, 0], xy[:, 1], "-o", ms=3, label="%s in %s" % (source, target))
            ax.legend(fontsize=self.font_size_legend)
        ax.set_title("tf trajectory: %d samples" % n, fontsize=self.font_size_title)
        ax.set_xlabel("x (m)", fontsize=self.font_size_axis_labels)
        ax.set_ylabel("y (m)", fontsize=self.font_size_axis_labels)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True)
        self._finish(show)

    # --- compilation -----------------------------------------------------
    def plot_compilation(self, idx=0, axs=None, show=False):
        """Radar cloud, 2D lidar, camera side by side (roles that are absent
        are labelled instead of raising)."""
        if axs is None or len(axs) == 0:
            fig, axs = plt.subplots(1, 3, figsize=(15, 5))
            fig.subplots_adjust(wspace=0.3)
        ds = self.dataset
        panels = [("radar_pc", self.plot_radar_detections),
                  ("lidar", self.plot_2d_lidar_detections),
                  ("camera", self.plot_camera_data)]
        for ax, (role, fn) in zip(axs, panels):
            if role in ds.streams:
                fn(idx, ax=ax, show=False)
            else:
                ax.set_title(role)
                ax.text(0.5, 0.5, "absent", ha="center", va="center",
                        transform=ax.transAxes)
        self._finish(show)
