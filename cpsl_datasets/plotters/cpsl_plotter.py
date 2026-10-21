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

    # --- time series -----------------------------------------------------
    def _series(self, ax, data, labels, title, ylabel, unit_note=""):
        ax = self._ax(ax)
        data = np.asarray(data)
        if len(data):
            for k, lab in enumerate(labels):
                ax.plot(data[:, 0], data[:, 1 + k], label=lab)
            ax.legend(fontsize=self.font_size_legend)
        ax.set_title("%s: %d samples%s" % (title, len(data), unit_note),
                     fontsize=self.font_size_title)
        ax.set_xlabel("t (s)", fontsize=self.font_size_axis_labels)
        ax.set_ylabel(ylabel, fontsize=self.font_size_axis_labels)
        ax.grid(True)
        return ax

    def _whole(self, role, cols):
        """``(m, 1+len(cols))`` ``t, cols`` of every stored row of a row stream."""
        ds = self.dataset
        ds._stream(role)
        s = ds.streams[role]
        data = np.asarray(s._g["data"][:])[:, cols]
        return np.c_[np.asarray(ds._t[role]) * 1e-9, data]

    def plot_imu(self, ax=None, show=False):
        """Gyro (rad/s) and accelerometer (m/s^2) of the whole IMU stream against time
        (two axes: pass ``ax`` as a length-2 sequence, else a new figure is made)."""
        from cpsl_datasets.cpsl_ds import _IMU_COLS
        if ax is None:
            ax = plt.subplots(2, 1, sharex=True)[1]
        for a, names, ylab in ((ax[0], ("wx", "wy", "wz"), "angular rate (rad/s)"),
                               (ax[1], ("ax", "ay", "az"), "acceleration (m/s^2)")):
            d = self._whole("imu", [_IMU_COLS.index(c) for c in names])
            self._series(a, d, names, "IMU", ylab)
        self._finish(show)

    def plot_vehicle_vel(self, ax=None, show=False):
        """Forward velocity vx (m/s) and yaw rate wz (rad/s) of the whole velocity stream."""
        from cpsl_datasets.cpsl_ds import VEL_COLS
        ds = self.dataset
        ds._stream("vehicle_vel")
        cols = VEL_COLS[ds.streams["vehicle_vel"].type]
        d = self._whole("vehicle_vel", list(cols))
        ax = self._series(ax, d, ("vx (m/s)", "wz (rad/s)"), "Vehicle velocity", "vx / wz")
        self._finish(show)
        return ax

    # --- hands -----------------------------------------------------------
    def plot_hands_3d(self, sample_idx, ax=None, show=False):
        """3D scatter of the Leap hand joints (left blue, right red) in the Leap
        sensor frame as stored (metres, raw Leap coordinates). An absent hand
        (all-zero placeholder) is not drawn and is noted in the legend."""
        ds = self.dataset
        if ax is None:
            ax = plt.figure().add_subplot(projection="3d")
        roles = [(r, h, c) for r, h, c in (("hand_left", "left", "tab:blue"),
                                           ("hand_right", "right", "tab:red"),
                                           ("hand", None, "tab:green")) if r in ds.streams]
        if not roles:
            raise ValueError("dataset has no hand stream")
        for _, h, c in roles:
            pts = ds.get_hand_tracking_data(sample_idx, h)
            name = h or "hand"
            if ds.hand_present(sample_idx, h):
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=c, label=name, s=self.marker_size)
            else:
                ax.scatter([], [], [], color=c, label=name + " (absent)")
        ax.set_xlabel("x (m, leap_link)", fontsize=self.font_size_axis_labels)
        ax.set_ylabel("y (m, leap_link)", fontsize=self.font_size_axis_labels)
        ax.set_zlabel("z (m, leap_link)", fontsize=self.font_size_axis_labels)
        ax.set_title("Hands (Leap sensor frame)", fontsize=self.font_size_title)
        ax.legend(fontsize=self.font_size_legend)
        self._finish(show)

    # --- Leap images -----------------------------------------------------
    def plot_leap_images(self, sample_idx, axs=None, show=False):
        """Left and right Leap camera images (mono8) side by side."""
        ds = self.dataset
        if axs is None or len(axs) == 0:
            axs = plt.subplots(1, 2, figsize=(10, 5))[1]
        for ax, role, fn in zip(axs, ("leap_left", "leap_right"),
                                (ds.get_leap_motion_image_frame_left,
                                 ds.get_leap_motion_image_frame_right)):
            ax.set_title(role.replace("_", " "))
            img = np.asarray(fn(sample_idx)) if role in ds.streams else np.empty((0, 0))
            if img.size == 0:
                ax.text(0.5, 0.5, "absent" if role not in ds.streams else "no frame",
                        ha="center", va="center", transform=ax.transAxes)
            else:
                ax.imshow(img, cmap="gray")
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
