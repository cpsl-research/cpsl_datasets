import os
import numpy as np
import matplotlib.image as img #not needed for ROS

#########################################################################
### Notes
### * Unless otherwise specified, the coordinate frame is in FLU (x-forward, y-left, z-up)
### * Note that all point cloud data is transformed into the robot base frame
###   (i.e., the lidar and radar data is not in the sensor frame)
#########################################################################
class CpslDS:

    def __init__(self,
                 dataset_path,
                 radar_folder="radar_0",
                 lidar_folder="lidar",
                 camera_folder="camera",
                 hand_tracking_folder="hand_tracking",
                 leap_motion_image_left_folder="leap_images/left",
                 leap_motion_image_right_folder="leap_images/right",
                 imu_orientation_folder="imu_data",
                 imu_full_folder="imu_full",
                 vehicle_vel_folder="vehicle_vel",
                 vehicle_odom_folder="vehicle_odom"

                 ) -> None:
        """Initializes the CpslDS class.

        This class provides an interface to access various sensor data from the CPSL datasets.

        Args:
            dataset_path (str): The path to the dataset directory.

        Keyword Args:
            radar_folder (str): The name of the folder containing the radar data. Defaults to "radar_0".
            lidar_folder (str): The name of the folder containing the lidar data. Defaults to "lidar".
            camera_folder (str): The name of the folder containing the camera data. Defaults to "camera".
            hand_tracking_folder (str): The name of the folder containing the hand tracking data. Defaults to "hand_tracking".
            leap_motion_image_left_folder (str): The name of the folder containing the left Leap Motion camera images. Defaults to "leap_images/left".
            leap_motion_image_right_folder (str): The name of the folder containing the right Leap Motion camera images. Defaults to "leap_images/right".
            imu_orientation_folder (str): The name of the folder containing the IMU orientation data. Defaults to "imu_data".
            imu_full_folder (str): The name of the folder containing the full IMU data. Defaults to "imu_full".
            vehicle_vel_folder (str): The name of the folder containing the vehicle velocity data. Defaults to "vehicle_vel".
            vehicle_odom_folder (str): The name of the folder containing the vehicle odometry data. Defaults to "vehicle_odom".

        Sensor Data Format:
            - Radar:
                - Point Cloud: (N, 4) numpy array with [x, y, z, velocity]
                - ADC Cube: (rx_channels, samples, chirps) complex numpy array
            - Lidar:
                - Point Cloud (filtered): (N, 2) numpy array with [x, y]
                - Point Cloud (raw): (N, 4) numpy array with [x, y, z, intensity]
            - Camera:
                - Image: numpy array with RGB channels
            - Hand Tracking:
                - Joints: (21, 3) numpy array with [x, y, z] for each joint
            - Leap Motion:
                - Image: numpy array representing the camera image
            - IMU:
                - Orientation: heading in radians (float)
                - Full Data: (N, 7) numpy array with [time, w_x, w_y, w_z, acc_x, acc_y, acc_z]
            - Vehicle:
                - Velocity: (N, 3) numpy array with [time, vx, wz]
                - Odometry: (N, 14) numpy array with [time, x, y, z, quat_w, quat_x, quat_y, quat_z, vx, vy, vz, wx, wy, wz]
        """
        
      
        #radar data
        self.radar_enabled = False
        self.radar_folder = radar_folder
        self.radar_files = []

        #lidar data
        self.lidar_enabled = False
        self.lidar_folder = lidar_folder
        self.lidar_files = []

        #camera data
        self.camera_enabled = False
        self.camera_folder = camera_folder
        self.camera_files = []

        #hand tracking data
        self.hand_tracking_enabled = False
        self.hand_tracking_folder = hand_tracking_folder
        self.hand_tracking_files = []

        #leap motion image data
        self.leap_motion_images_enabled = False
        self.leap_motion_image_left_folder = leap_motion_image_left_folder
        self.leap_motion_image_right_folder = leap_motion_image_right_folder
        self.leap_motion_image_left_files = []
        self.leap_motion_image_right_files = []

        #imu data - orientation only
        self.imu_orientation_folder = imu_orientation_folder
        self.imu_orientation_files = []
        self.imu_orientation_enabled = False

        #imu data - full sensor data
        self.imu_full_folder = imu_full_folder
        self.imu_full_files = []
        self.imu_full_enabled = False

        #vehicle velocity data
        self.vehicle_vel_folder = vehicle_vel_folder
        self.vehicle_vel_files = []
        self.vehicle_vel_enabled = False

        #vehicle odometry data
        self.vehicle_odom_folder = vehicle_odom_folder
        self.vehicle_odom_files = []
        self.vehicle_odom_enabled = False

        #variable to keep track of the number of frames
        self.num_frames = 0

        #load the new dataset
        self.load_new_dataset(dataset_path)

        return

    def load_new_dataset(self,dataset_path:str):

        self.dataset_path = dataset_path
        self.import_dataset_files()
        self.determine_num_frames()

    def import_dataset_files(self):

        self.import_radar_data()
        self.import_lidar_data()
        self.import_camera_data()
        self.import_hand_tracking_data()
        self.import_leap_motion_image_data()
        self.import_imu_orientation_data()
        self.import_imu_full_data()  
        self.import_vehicle_vel_data()
        self.import_vehicle_odom_data()
            
    def determine_num_frames(self):

        self.num_frames = 0

        if self.radar_enabled:
            self.set_num_frames(len(self.radar_files))
        if self.lidar_enabled:
            self.set_num_frames(len(self.lidar_files))
        if self.camera_enabled:
            self.set_num_frames(len(self.camera_files))
        if self.hand_tracking_enabled:
            self.set_num_frames(len(self.hand_tracking_files))
        if self.leap_motion_images_enabled:
            self.set_num_frames(len(self.leap_motion_image_left_files))
        if self.imu_full_enabled:
            self.set_num_frames(len(self.imu_full_files))
        if self.imu_orientation_enabled:
            self.set_num_frames(len(self.imu_orientation_files))
        if self.vehicle_vel_enabled:
            self.set_num_frames(len(self.vehicle_vel_files))
        
        return
    
    def set_num_frames(self,num_files:int):
        """Update the number of frames available in the dataset

        Args:
            num_files (int): The number of files available for a given sensor
        """
        if self.num_frames > 0:
            self.num_frames = min(self.num_frames,num_files)
        else:
            self.num_frames = num_files
    
    ####################################################################
    #handling radar data
    ####################################################################   
    def import_radar_data(self):

        path = os.path.join(self.dataset_path,self.radar_folder)

        if os.path.isdir(path):
            self.radar_enabled = True
            self.radar_files = sorted(os.listdir(path))
            print("found {} radar samples".format(len(self.radar_files)))
        else:
            print("did not find radar samples")

        return
    
    def get_radar_data(self,idx:int)->np.ndarray:
        """Get radar detections or ADC data cube for a specific index in the dataset.
        The format of the returned data depends on whether the raw ADC data or the processed point cloud is available.
        The radar point cloud data is stored in the base frame of a robot (i.e. it may not be in the radar's coordinate frame)

        - If radar point cloud data is available, the format is a numpy array of shape (N, 4), where N is the number of detections. The columns are:
            - x (float): x-coordinate in meters
            - y (float): y-coordinate in meters
            - z (float): z-coordinate in meters
            - vel (float): velocity in m/s

        - If raw ADC data is available, the format is a numpy array of shape (rx_channels, samples, chirps) with complex values.

        Args:
            idx (int): The index of the radar data detection

        Returns:
            np.ndarray: An Nx4 of radar detections with (x,y,z,vel) vals or
                        (rx_channels) x (samples) x (chirps) ADC cube for a given frame
        """

        assert self.radar_enabled, "No radar dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.radar_folder,
            self.radar_files[idx])
        
        points = np.load(path)
                
        return points
    
    ####################################################################
    #handling lidar data
    ####################################################################   
    def import_lidar_data(self):

        path = os.path.join(self.dataset_path,self.lidar_folder)

        if os.path.isdir(path):
            self.lidar_enabled = True
            self.lidar_files = sorted(os.listdir(path))
            print("found {} lidar samples".format(len(self.lidar_files)))
        else:
            print("did not find lidar samples")

        return
    
    def get_lidar_point_cloud(self,idx)->np.ndarray:
        """Get a lidar pointcloud from the desired frame, filters out ground and higher detections points.

        The raw lidar data is a point cloud with (x, y, z, intensity) values. This function filters the raw data to remove the ground and high-elevation points, returning only the (x, y) coordinates.

        Returns:
            np.ndarray: a Nx2 array of lidar detections (x,y)
        """
        assert self.lidar_enabled, "No lidar dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.lidar_folder,
            self.lidar_files[idx]
        )
        
        points = np.load(path)

        valid_points = points[:,2] > -0.2 #filter out ground
        valid_points = valid_points & (points[:,2] < 0.1) #higher elevation points

        points = points[valid_points,:2]

        return points
    
    def get_lidar_point_cloud_raw(self,idx)->np.ndarray:
        """Get a lidar pointcloud from the desired frame, without filtering anything out.

        The format of the returned data is a numpy array of shape (N, 4), where N is the number of points. The columns are:
            - x (float): x-coordinate in meters
            - y (float): y-coordinate in meters
            - z (float): z-coordinate in meters
            - intensity (float): intensity of the point

        Returns:
            np.ndarray: a Nx4 array of lidar detections (x,y,z,intensity)
        """

        assert self.lidar_enabled, "No lidar dataset loaded"
        
        path = os.path.join(
            self.dataset_path,
            self.lidar_folder,
            self.lidar_files[idx]
        )
        
        
        points = np.load(path)

        return points
        
    ####################################################################
    #handling camera data
    ####################################################################   
    def import_camera_data(self):

        path = os.path.join(self.dataset_path,self.camera_folder)

        if os.path.isdir(path):
            self.camera_enabled = True
            self.camera_files = sorted(os.listdir(path))
            print("found {} camera samples".format(len(self.camera_files)))
        else:
            print("did not find camera samples")

        return

    def get_camera_frame(self,idx:int)->np.ndarray:
        """Get a camera frame from the dataset

        Args:
            idx (int): the index in the dataset to get the camera
                data from

        Returns:
            np.ndarray: the camera data with rgb channels
        """
        assert self.camera_enabled, "No camera dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.camera_folder,
            self.camera_files[idx])
        image = img.imread(path)

        #return while also flipping red and blue channel
        # return image[:,:,::-1]
        return image
    
    ####################################################################
    #handling hand tracking data
    ####################################################################   
    def import_hand_tracking_data(self):

        path = os.path.join(self.dataset_path,self.hand_tracking_folder)

        if os.path.isdir(path):
            self.hand_tracking_enabled = True
            self.hand_tracking_files = sorted(os.listdir(path))
            print("found {} hand tracking samples".format(len(self.hand_tracking_files)))
        else:
            print("did not find hand tracking samples")

        return

    def get_hand_tracking_data(self,idx:int)->np.ndarray:
        """Get hand tracking data from the dataset.

        The format of the returned data is a numpy array of shape (21, 3), where each row represents a joint and the columns are the x, y, and z coordinates.

        The joints are ordered as follows:
            - 0: Palm position
            - 1-4: Thumb (metacarpal, proximal, intermediate, distal)
            - 5-8: Index finger (metacarpal, proximal, intermediate, distal)
            - 9-12: Middle finger (metacarpal, proximal, intermediate, distal)
            - 13-16: Ring finger (metacarpal, proximal, intermediate, distal)
            - 17-20: Pinky finger (metacarpal, proximal, intermediate, distal)

        Args:
            idx (int): the index in the dataset to get the hand tracking
                data from

        Returns:
            np.ndarray: the hand tracking data
        """
        assert self.hand_tracking_enabled, "No hand tracking dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.hand_tracking_folder,
            self.hand_tracking_files[idx]
        )
        data = np.load(path)

        return data
    
    ####################################################################
    #handling leap_motion data
    ####################################################################   
    def import_leap_motion_image_data(self):

        #handle left images
        path = os.path.join(self.dataset_path,self.leap_motion_image_left_folder)

        if os.path.isdir(path):
            self.leap_motion_images_enabled = True
            self.leap_motion_image_left_files = sorted(os.listdir(path))
            print("found {} leap motion left samples".format(len(self.leap_motion_image_left_files)))
        else:
            print("did not find leap motion left samples")
        
        #handle right images
        path = os.path.join(self.dataset_path,self.leap_motion_image_right_folder)

        if os.path.isdir(path):
            self.leap_motion_images_enabled = True
            self.leap_motion_image_right_files = sorted(os.listdir(path))
            print("found {} leap motion right samples".format(len(self.leap_motion_image_right_files)))
        else:
            print("did not find leap motion right samples")

        return

    def get_leap_motion_image_frame_left(self,idx:int)->np.ndarray:
        """Get a leap motion left camera frame from the dataset

        Args:
            idx (int): the index in the dataset to get the leap motion
        """

        assert self.leap_motion_images_enabled, "No leap motion dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.leap_motion_image_left_folder,
            self.leap_motion_image_left_files[idx])
        image = img.imread(path)

        return image
    
    def get_leap_motion_image_frame_right(self,idx:int)->np.ndarray:
        """Get a leap motion right camera frame from the dataset

        Args:
            idx (int): the index in the dataset to get the leap motion
        """
        assert self.leap_motion_images_enabled, "No leap motion dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.leap_motion_image_right_folder,
            self.leap_motion_image_right_files[idx])
        image = img.imread(path)

        return image

    ####################################################################
    #handling imu data (orientation only)
    ####################################################################   
    def import_imu_orientation_data(self):

        path = os.path.join(self.dataset_path,self.imu_orientation_folder)

        if os.path.isdir(path):
            self.imu_orientation_enabled = True
            self.imu_orientation_files = sorted(os.listdir(path))
            print("found {} imu (orientation only) samples".format(len(self.imu_orientation_files)))
        else:
            print("did not find imu (orientation) samples")

        return
    
    def get_imu_orientation_rad(self,idx:int):
        """Get the raw imu heading from the dataset at a given frame index.

        The data is loaded from a file containing a quaternion [w, x, y, z] and the heading is calculated from it.

        Args:
            idx (int): the frame index to get the imu heading for

        Returns:
            _type_: the raw heading read from the IMU expressed in the range
                [-pi,pi]
        """
        assert self.imu_orientation_enabled, "No IMU (orientation) dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.imu_orientation_folder,
            self.imu_orientation_files[idx]
        )

        data = np.load(path)
        w = data[0]
        x = data[1]
        y = data[2]
        z = data[3]

        heading = np.arctan2(
            2 * (w * z + x * y), 1 - 2 * (y * y + z * z)
        )
        return heading
    
    ####################################################################
    #handling imu (full sensor) data
    ####################################################################   
    def import_imu_full_data(self):

        path = os.path.join(self.dataset_path,self.imu_full_folder)

        if os.path.isdir(path):
            self.imu_full_enabled = True
            self.imu_full_files = sorted(os.listdir(path))
            print("found {}imu (full data) samples".format(len(self.imu_full_files)))
        else:
            print("did not find imu (full data) samples")

        return
    
    def get_imu_full_data(self,idx=0):
        """Get the full IMU data from the dataset.

        The format of the returned data is a numpy array of shape (N, 7), where N is the number of measurements. The columns are:
            - time (float): timestamp in seconds
            - w_x (float): angular velocity about x-axis in rad/s
            - w_y (float): angular velocity about y-axis in rad/s
            - w_z (float): angular velocity about z-axis in rad/s
            - acc_x (float): linear acceleration along x-axis in m/s^2
            - acc_y (float): linear acceleration along y-axis in m/s^2
            - acc_z (float): linear acceleration along z-axis in m/s^2

        Args:
            idx (int, optional): The index of the IMU data. Defaults to 0.

        Returns:
            np.ndarray: [time,w_x,w_y,w_z,acc_x,acc_y,acc_z]
        """
        assert self.imu_full_enabled, "No IMU Full dataset loaded"

        #load the data sample
        path = os.path.join(
            self.dataset_path,
            self.imu_full_folder,
            self.imu_full_files[idx])

        return np.load(path)
    
    ####################################################################
    #handling vehicle velocity data
    ####################################################################
    def import_vehicle_vel_data(self):

        path = os.path.join(self.dataset_path,self.vehicle_vel_folder)

        if os.path.isdir(path):
            self.vehicle_vel_enabled = True
            self.vehicle_vel_files = sorted(os.listdir(path))
            print("found {} vehicle velocity samples".format(len(self.vehicle_vel_files)))
        else:
            print("did not find vehicle velocity samples")

        return
    
    def get_vehicle_vel_data(self,idx=0):
        """Get the vehicle velocity data from the dataset.

        The format of the returned data is a numpy array of shape (N, 3), where N is the number of measurements. The columns are:
            - time (float): timestamp in seconds
            - vx (float): linear velocity along x-axis in m/s
            - wz (float): angular velocity about z-axis in rad/s

        Args:
            idx (int, optional): The index of the vehicle velocity data. Defaults to 0.

        Returns:
            np.ndarray: [time, vx, wz]
        """

        assert self.vehicle_vel_files, "No Vehicle velocity dataset loaded"

        #load the data sample
        path = os.path.join(
            self.dataset_path,
            self.vehicle_vel_folder,
            self.vehicle_vel_files[idx])

        return np.load(path)
    
    ####################################################################
    #handling vehicle odometry data
    ####################################################################
    def import_vehicle_odom_data(self):

        path = os.path.join(self.dataset_path,self.vehicle_odom_folder)

        if os.path.isdir(path):
            self.vehicle_odom_enabled = True
            self.vehicle_odom_files = sorted(os.listdir(path))
            print("found {} vehicle odometry samples".format(len(self.vehicle_odom_files)))
        else:
            print("did not find vehicle odometry samples")

        return
    
    def get_vehicle_odom_data(self,idx=0):
        """Get the vehicle odometry data from the dataset.

        The format of the returned data is a numpy array of shape (N, 14), where N is the number of measurements. The columns are:
            - time (float): timestamp in seconds
            - x (float): x-coordinate in meters
            - y (float): y-coordinate in meters
            - z (float): z-coordinate in meters
            - quat_w (float): w component of the orientation quaternion
            - quat_x (float): x component of the orientation quaternion
            - quat_y (float): y component of the orientation quaternion
            - quat_z (float): z component of the orientation quaternion
            - vx (float): linear velocity along x-axis in m/s
            - vy (float): linear velocity along y-axis in m/s
            - vz (float): linear velocity along z-axis in m/s
            - wx (float): angular velocity about x-axis in rad/s
            - wy (float): angular velocity about y-axis in rad/s
            - wz (float): angular velocity about z-axis in rad/s

        Args:
            idx (int, optional): The index of the vehicle odometry data. Defaults to 0.

        Returns:
            np.ndarray: [time,x,y,z,quat_w,quat_x,quat_y,quat_z,vx,vy,vz,wx,wy,wz]
        """

        assert self.vehicle_odom_enabled, "No Vehicle odometry dataset loaded"

        #load the data sample
        path = os.path.join(
            self.dataset_path,
            self.vehicle_odom_folder,
            self.vehicle_odom_files[idx])

        return np.load(path)