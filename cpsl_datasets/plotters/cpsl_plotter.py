import matplotlib.pyplot as plt
import numpy as np

from cpsl_datasets.cpsl_ds import CpslDS

#import geometries for help
from geometries.pose.orientation import Orientation
from geometries.pose.pose import Pose
from geometries.transforms.transformation import Transformation

class CpslPlotter:

    def __init__(self,dataset:CpslDS,) -> None:
        
        #define default plot parameters:
        self.font_size_axis_labels = 12
        self.font_size_title = 15
        self.font_size_ticks = 12
        self.font_size_legend = 12
        self.plot_x_max = 20
        self.plot_y_max = 20
        self.marker_size = 10

        #particle filter specific
        self.particle_marker_size = 5
        self.arrow_length = 0.4
        self.particles_x_buffer = 5
        self.particles_y_buffer = 5       

        #import the dataset
        self.dataset:CpslDS = dataset

        return
    
    def _plot_detections(
            self,
            current_points:np.ndarray,
            ax:plt.Axes=None,
            show=False
    ):
        """Plots a point cloud (+x is forward, +y is left)

        Args:
            current_points (np.ndarray): Nx3 array with  x,y,z point cloud in agent frame
            ax (plt.Axes, optional): A set of axes to plot on. 
                Defaults to None.
            show (bool, optional): on True, shows the plot. 
                Defaults to False.
        """
        
        if not ax:
            fig,ax = plt.subplots()

        #plot the aligned_detections
        ax.scatter(
            current_points[:,0],
            current_points[:,1],
            label="detections",
            marker="D",
            color="red",
            s=self.marker_size)
        

        ax.set_title("Point cloud Detections: {}".format(current_points.shape[0]),fontsize=self.font_size_title)
        ax.set_xlim(
            - self.plot_x_max,
            self.plot_x_max)
        ax.set_xlabel("X",fontsize=self.font_size_axis_labels)
        ax.set_ylim(
            - self.plot_y_max,
            + self.plot_y_max)
        ax.set_ylabel("Y",fontsize=self.font_size_axis_labels)
        ax.tick_params(labelsize=self.font_size_ticks)
        ax.xaxis.set_major_locator(plt.MultipleLocator(5.0))
        ax.yaxis.set_major_locator(plt.MultipleLocator(5.0))
        ax.grid("True")
        handles,labels = ax.get_legend_handles_labels()
        ax.legend(handles[1:3], labels[1:3], loc="lower right",fontsize=self.font_size_legend)

        if show:
            plt.show()

        return
    
    def plot_radar_detections(
        self,
        sample_idx:int,
        ax:plt.Axes = None,
        show=False
    ):  
        """Plot the radar detections (assuming point cloud data)

        Args:
            sample_idx (int): The sample index from the dataset to plot
            ax (plt.Axes, optional): _description_. Defaults to None.
            show (bool, optional): _description_. Defaults to False.
        """
        #get the x,y coordinates to plot
        radar_data = self.dataset.get_radar_point_cloud(idx=sample_idx)[:,0:3]

        #rotate the points suth that they can be plotted in FLU reference frame
        rotation = Orientation.from_euler(
            yaw=90,
            degrees=True
        )
        transformation = Transformation(
            rotation=rotation._orientation
        )
        radar_data = transformation.apply_transformation(radar_data)

        self._plot_detections(
            current_points=radar_data,
            ax=ax,
            show=show
        )
    
        return
    
    def plot_2d_lidar_detections_velodyne(
        self,
        sample_idx:int,
        ax:plt.Axes = None,
        show=False
    ):  
        """Plot the 2d lidar detections (assuming point cloud data)

        Args:
            sample_idx (int): The sample index from the dataset to plot
            ax (plt.Axes, optional): _description_. Defaults to None.
            show (bool, optional): _description_. Defaults to False.
        """
        #get the x,y coordinates to plot
        lidar_data = self.dataset.get_lidar_point_cloud(idx=sample_idx)
        zero_col = np.zeros((lidar_data.shape[0],1))
        lidar_data = np.hstack([lidar_data,zero_col])

        #rotate the points suth that they can be plotted in FLU reference frame
        rotation = Orientation.from_euler(
            yaw=90,
            degrees=True
        )
        transformation = Transformation(
            rotation=rotation._orientation
        )
        lidar_data = transformation.apply_transformation(lidar_data)

        self._plot_detections(
            current_points=lidar_data,
            ax=ax,
            show=show
        )
    
        return

    def plot_camera_data(
        self,
        sample_idx:int,
        ax:plt.Axes = None,
        show=False
    ):  
        """Plot the camera data

        Args:
            sample_idx (int): The sample index from the dataset to plot
            ax (plt.Axes, optional): _description_. Defaults to None.
            show (bool, optional): _description_. Defaults to False.
        """
        #get the x,y coordinates to plot
        camera_data = self.dataset.get_camera_frame(idx=sample_idx)

        if not ax:
            fig,ax = plt.subplots()

        ax.imshow(camera_data)
        ax.set_title("Camera View")

        if show:
            plt.show()
    
        return
    
    def plot_compilation(
            self,
            idx=0,
            axs:plt.Axes=[],
            show=False
        ):

        if len(axs) == 0:
            fig,axs=plt.subplots(1,3, figsize=(15,5)) #(W,H)
            fig.subplots_adjust(wspace=0.3,hspace=0.30)

        #top row pose(localization and heading) and camera view
        self.plot_radar_detections(
            sample_idx=idx,
            ax=axs[0],
            show=False
        )

        self.plot_2d_lidar_detections_velodyne(
            sample_idx=idx,
            ax=axs[1],
            show=False
        )

        self.plot_camera_data(
            sample_idx=idx,
            ax=axs[2],
            show=False
        )
        
        if show:

            plt.show()
