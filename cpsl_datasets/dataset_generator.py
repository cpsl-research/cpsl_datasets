import rclpy
from rclpy.node import Node
import rclpy.duration
from rclpy.node import Node,ParameterDescriptor,ParameterType
import rclpy.publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.time
from rclpy.time import Time
import rclpy.timer
from sensor_msgs.msg import PointCloud2,Image,Imu
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from raw_radar_msgs.msg import ADCDataCube

from geometry_msgs.msg import Vector3Stamped,TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import numpy as np
import json
import os

msg = Odometry()
msg.twist

class DatasetGenerator(Node):
    def __init__(self):
        super().__init__('dataset_generator')

        #parameters
        self.radar_enable:bool = False
        self.lidar_enable:bool = False
        self.lidar_topic:str = ""
        self.camera_enable:bool = False
        self.camera_topic:str = ""
        self.imu_enable:bool = False
        self.imu_topic:str = ""
        self.vicon_enable:bool = False
        self.vehicle_odom_enable:bool = False
        self.vehicle_odom_topic:str = ""
        self.base_frame:str = ""
        self.frame_rate_save_data:float = 10
        self.frame_rate_high_speed_sensors:float = 20
        self.dataset_path:str = ""

        #radar dataset generation
        self.radar_pc_topics:list = []
        self.radar_pc_folders:list = []
        self.radar_raw_adc_topics:list = []
        self.radar_raw_adc_folders:list = []

        #vicon dataset generation
        self.vicon_topics = [] # names for all of the publishing vicon topics (excluding the "marker" topic)
        self.vicon_data_folders = [] #capture data folder for all active vicon publishing

        #dataset generation support variables
        self.lidar_data_folder = "lidar"
        self.camera_data_folder = "camera"
        self.imu_data_folder = "imu_data" #position reading
        self.vehicle_odom_folder = "vehicle_odom"
        self.vehicle_vel_folder = "vehicle_vel"
        self.save_file_name = "frame"
        self.sample_idx = 10000 #start at frame 10000

        #defining qos profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        #subscriber lists (for multiple radars and multiple vicon targets)
        self.radar_pc_subs:list = []
        self.radar_raw_adc_subs:list = []
        self.vicon_subs:list = []

        #capturing latest messages
        self.radar_pc_msgs_latest:list = []
        self.radar_raw_adc_msgs_latest:list = []
        self.lidar_data_msg_latest:PointCloud2 = None
        self.camera_msg_latest:Image = None
        self.imu_data_msg_latest:Imu = None
        self.imu_data_buffer = [] #list of imu messages recorded at higher rate
        self.vicon_msgs_latest:list = []
        self.vehicle_odom_msg_latest:Odometry = None
        self.vehicle_odom_data_buffer = []

        #tf tree management
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self)

        #timers
        self.low_rate_timer:rclpy.timer.Timer = None
        self.high_rate_timer:rclpy.timer.Timer = None

        #initialize the node
        self.init_params() #parameters
        self.init_data_folders()
        self.init_subs()
        self.init_timers()

    ####################################################################
    # Initializing parameters
    ####################################################################

    def init_params(self):
        """Declare and set all of the ROS2 parameters
        """
        # Declare parameters
        self.declare_parameter(
            name='radar_enable',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable radar data streaming'
            )
        )
        self.declare_parameter(
            name='lidar_enable',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable lidar data streaming'
            )
        )
        self.declare_parameter(
            name='lidar_topic',
            value="livox/lidar",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='the topic to stream lidar data from'
            )
        )
        self.declare_parameter(
            name='camera_enable',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable camera data streaming'
            )
        )
        self.declare_parameter(
            name='camera_topic',
            value="usb_cam/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The topic to stream camera data from'
            )
        )
        self.declare_parameter(
            name='imu_enable',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable imu data streaming'
            )
        )
        self.declare_parameter(
            name='imu_topic',
            value="imu/data",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='the topic to stream imu data from'
            )
        )
        self.declare_parameter(
            name='vicon_enable',
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable camera data streaming'
            )
        )
        self.declare_parameter(
            name='vehicle_odom_enable',
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether or not to enable vehicle odometry data streaming'
            )
        )
        self.declare_parameter(
            name='vehicle_odom_topic',
            value="odom",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='the topic to stream vehicle odometry data from'
            )
        )
        self.declare_parameter(
            name='base_frame',
            value='base_link',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The base_link frame id to transform all point into'
            )
        )
        self.declare_parameter(
            name='frame_rate_save_data',
            value=10.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The rate in Hz to capture standard measurements at'
            )
        )
        self.declare_parameter(
            name='frame_rate_high_speed_sensors',
            value=20.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The rate in Hz to capture high rate sensors at'
            )
        )
        self.declare_parameter(
            name='dataset_path',
            value='/home/cpsl/data',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The path to where the datset will be created'
            )
        )
        
        #set parameters
        self.radar_enable = self.get_parameter('radar_enable').get_parameter_value().bool_value
        self.lidar_enable = self.get_parameter('lidar_enable').get_parameter_value().bool_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.camera_enable = self.get_parameter('camera_enable').get_parameter_value().bool_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.imu_enable = self.get_parameter('imu_enable').get_parameter_value().bool_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.vicon_enable = self.get_parameter('vicon_enable').get_parameter_value().bool_value
        self.vehicle_odom_enable = self.get_parameter('vehicle_odom_enable').get_parameter_value().bool_value
        self.vehicle_odom_topic = self.get_parameter('vehicle_odom_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.frame_rate_save_data = self.get_parameter('frame_rate_save_data').get_parameter_value().double_value
        self.frame_rate_high_speed_sensors = self.get_parameter('frame_rate_high_speed_sensors').get_parameter_value().double_value
        self.dataset_path = self.get_parameter('dataset_path').get_parameter_value().string_value

        #log the parameter values
        self.get_logger().info(f'radar_enable set to: {self.radar_enable}')
        self.get_logger().info(f'lidar_enable set to: {self.lidar_enable}')
        self.get_logger().info(f'camera_enable set to: {self.camera_enable}')
        self.get_logger().info(f'imu_enable set to: {self.imu_enable}')
        self.get_logger().info(f'vicon_enable set to: {self.vicon_enable}')
        self.get_logger().info(f'vehicle_odom_enable set to: {self.vehicle_odom_enable}')
        self.get_logger().info(f'base_frame set to: {self.base_frame}')
        self.get_logger().info(f'frame_rate set to: {self.frame_rate_save_data}')
        self.get_logger().info(f'frame_rate_high_speed_sensors set to: {self.frame_rate_high_speed_sensors}')
        self.get_logger().info(f'dataset_path set to: {self.dataset_path}')

        

    ####################################################################
    # Initializing dataset folders
    ####################################################################

    def check_for_directory(self, path, clear_contents=False):
        """Checks if a directory exists. If not, creates it.
        Optionally clears contents if directory exists.

        Args:
            path (str): Path to the directory to create/check.
            clear_contents (bool, optional): Removes all contents if True. Defaults to False.
        """

        if os.path.isdir(path):
            self.get_logger().info(f"check_for_directory: Found directory {path}")

            if clear_contents:
                self.get_logger().info(f"check_for_directory: Clearing contents of {path}")
                self.clear_directory(path)
        else:
            self.get_logger().info(f"check_for_directory: Creating directory {path}")
            os.makedirs(path)

        return
    
    def clear_directory(self,path_to_clean):
        for entry in os.listdir(path_to_clean):
            full_path = os.path.join(path_to_clean, entry)
            try:
                if os.path.isfile(full_path):
                    os.remove(full_path)
                elif os.path.isdir(full_path):
                    self.clear_directory(full_path)  # Recursively clear contents
                    os.rmdir(full_path)    # Then remove the empty directory
            except Exception as e:
                self.get_logger().error(f"Failed to delete {full_path}: {e}")
        
    def init_data_folders(self):

        #initialize the top level folder
        self.check_for_directory(self.dataset_path,clear_contents=True)

        #initialize the lidar data folder
        if self.lidar_enable:
            path = os.path.join(self.dataset_path,self.lidar_data_folder)
            self.check_for_directory(path,clear_contents=True)

        if self.camera_enable:
            path = os.path.join(self.dataset_path,self.camera_data_folder)
            self.check_for_directory(path,clear_contents=True)

        if self.imu_enable:
            
            #imu data full (all IMU data)
            path = os.path.join(self.dataset_path,self.imu_data_folder)
            self.check_for_directory(path,clear_contents=True)

        #initialize the radar data folders
        if self.radar_enable:
            self.radar_data_folders_init()
        
        if self.vicon_enable:
            self.vicon_data_folders_init()
        
        if self.vehicle_odom_enable:
            #full odom
            path = os.path.join(self.dataset_path,self.vehicle_odom_folder)
            self.check_for_directory(path,clear_contents=True)

            #vehicle velocity data for backwards compatibility
            path = os.path.join(self.dataset_path,self.vehicle_vel_folder)
            self.check_for_directory(path,clear_contents=True)

    def vicon_data_folders_init(self):
        # Get a list of all topics and their types
        topic_list = self.get_topic_names_and_types()

        # Filter for Vicon topics
        vicon_topics = [name for name, _ in topic_list if "vicon" in name and "markers" not in name]
        self.vicon_topics = vicon_topics

        if len(self.vicon_topics) == 0:
            self.get_logger().info("No Vicon topics found")
        else:
            self.get_logger().info(f"Vicon topics: {self.vicon_topics}")

        # Initialize folders for each vicon object
        for topic in self.vicon_topics:
            object_name = topic.split("/")[-1]
            folder_name = f"vicon_{object_name}"

            path = os.path.join(self.dataset_path, folder_name)
            self.check_for_directory(path, clear_contents=True)

            self.vicon_data_folders.append(folder_name)
    
    def radar_data_folders_init(self):
        # Get all topic names and types
        topic_list = self.get_topic_names_and_types()

        #Find radar point cloud topics
        self.radar_pc_topics = [name for name, _ in topic_list if "radar" in name.lower() and "detected_points" in name.lower()]
        self.radar_pc_folders = ["{}_pc".format(name.split("/")[-2]) for name in self.radar_pc_topics]
        if len(self.radar_pc_topics) == 0:
            self.get_logger().info("No point cloud topics found.")
        else:
            self.get_logger().info("Radar point cloud topics: {}".format(self.radar_pc_topics))
            for folder in self.radar_pc_folders:
                path = os.path.join(self.dataset_path, folder)
                self.check_for_directory(path, clear_contents=True)

        #Find raw radar data topics
        self.radar_raw_adc_topics = [name for name, _ in topic_list if "radar" in name.lower() and "adc_data_cube" in name.lower()]
        self.radar_raw_adc_folders = ["{}_adc".format(name.split("/")[-2]) for name in self.radar_raw_adc_topics]
        if len(self.radar_raw_adc_topics) == 0:
            self.get_logger().info("No raw adc data topics found")
        else:
            self.get_logger().info("Radar raw adc data topics: {}".format(self.radar_raw_adc_topics))
            for folder in self.radar_raw_adc_folders:
                path = os.path.join(self.dataset_path, folder)
                self.check_for_directory(path, clear_contents=True)


    ####################################################################
    # Initializing subscribers
    ####################################################################
    def init_subs(self):
        """Initialize the point cloud subscribers based on the given in_point_cloud_topics list
        """
        if self.radar_enable:
            self.init_radar_subs()
        if self.lidar_enable:
            self.create_subscription(
                msg_type=PointCloud2,
                topic=self.lidar_topic,
                callback=self.lidar_sub_callback,
                qos_profile=self.qos_profile
            )
        if self.camera_enable:
            self.create_subscription(
                msg_type=Image,
                topic=self.camera_topic,
                callback=self.camera_sub_callback,
                qos_profile=self.qos_profile
            )
        if self.imu_enable:
            self.create_subscription(
                msg_type=Imu,
                topic=self.imu_topic,
                callback=self.imu_sub_callback,
                qos_profile=self.qos_profile
            )
        if self.vehicle_odom_enable:
            self.create_subscription(
                msg_type=Odometry,
                topic=self.vehicle_odom_topic,
                callback=self.vehicle_odom_sub_callback,
                qos_profile=self.qos_profile
            )
        if self.vicon_enable:
            self.init_vicon_subs()
    
    def init_radar_subs(self):

        #initialize point cloud subscribers
        for i in range(len(self.radar_pc_topics)):
            sub = self.create_subscription(
                msg_type=PointCloud2,
                topic=self.radar_pc_topics[i],
                callback=lambda msg, idx=i: self.radar_pc_sub_callback(msg,idx),
                qos_profile=self.qos_profile
            )
            self.radar_pc_subs.append(sub)
            self.radar_pc_msgs_latest.append(None) #append empty message for now
        
        #initialize raw adc data subscribers
        for i in range(len(self.radar_raw_adc_topics)):
            sub = self.create_subscription(
                msg_type=ADCDataCube,
                topic=self.radar_raw_adc_topics[i],
                callback=lambda msg, idx=i: self.radar_raw_sub_callback(msg,idx),
                qos_profile=self.qos_profile
            )
            self.radar_raw_adc_subs.append(sub)
            self.radar_raw_adc_msgs_latest.append(None) #append empty message for now
    
    def init_vicon_subs(self):

        #initialize vicon subscribers
        for i in range(len(self.vicon_topics)):
            sub = self.create_subscription(
                msg_type=TransformStamped,
                topic=self.vicon_topics[i],
                callback=lambda msg, idx=i: self.vicon_sub_callback(msg,idx),
                qos_profile=self.qos_profile
            )
            self.vicon_subs.append(sub)
            self.vicon_msgs_latest.append(None) #append empty message for now
        
    ####################################################################
    # Subscriber callbacks
    ####################################################################
    def radar_pc_sub_callback(self,msg:PointCloud2,index:int):
        """Update the radar-pc_msgs_latest array with the latest point cloud message from a given array

        Args:
            msg (PointCloud2): A PointCloud2 message to add to the array
            index (int): the index of the point cloud subscriber corresponding to the message
        """
        self.radar_pc_msgs_latest[index] = msg
        return
    
    def radar_raw_sub_callback(self,msg:ADCDataCube,index:int):
        """Update the radar_raw_adc_msgs_latest array with the latest point cloud message from a given array

        Args:
            msg (ADCDataCube): An ADCDataCube message to add to the array
            index (int): the index of the subscriber corresponding to the message
        """
        self.radar_raw_adc_msgs_latest[index] = msg
        return
    
    def lidar_sub_callback(self,msg:PointCloud2):
        """Update the lidar_data_msg_latest with the latest message from the lidar

        Args:
            msg (PointCloud2): An PointCloud2 message to add to the array
        """
        self.lidar_data_msg_latest = msg
        return
    
    def camera_sub_callback(self,msg:Image):
        """Update the camera_data_msg_latest with the latest message from the camera

        Args:
            msg (Image): An Image message to save
        """
        self.camera_msg_latest = msg

        return
    
    def camera_sub_callback(self,msg:Image):
        """Update the camera_data_msg_latest with the latest message from the camera

        Args:
            msg (Image): An Image message to save
        """
        self.camera_msg_latest = msg
        
        return
    
    def imu_sub_callback(self,msg:Imu):
        """Update the imu_data_msg_latest with the latest message from the IMU

        Args:
            msg (IMU): An IMU message to save
        """
        self.imu_data_msg_latest = msg
        
        return
    
    def vicon_sub_callback(self,msg:TransformStamped,index:int):
        """Update the vicon_msgs_latest array with the latest point cloud message from a given array

        Args:
            msg (TransformStamped): A TransformStamped message to add to the array
            index (int): the index of the vicon subscriber corresponding to the message
        """
        self.vicon_msgs_latest[index] = msg
        return
    
    def vehicle_odom_sub_callback(self,msg:Odometry):
        """Update the vehicle_odom_msg_latest with the latest message from the IMU

        Args:
            msg (Odometry): An Odometry message to save
        """
        self.vehicle_odom_msg_latest = msg
        
        return

    ####################################################################
    # Timer functions
    ####################################################################
    def init_timers(self): 

        self.low_rate_timer = self.create_timer(
            timer_period_sec= (1/self.frame_rate_save_data),
            callback=self.save_latest_data
        )

        self.high_rate_timer = self.create_timer(
            timer_period_sec= (1/self.frame_rate_high_speed_sensors),
            callback=self.update_high_rate_sensors
        )

    ####################################################################
    # Timer callback functions
    ####################################################################
    def save_latest_data(self):

        if self.check_latest_data():

            if self.radar_enable:
                self.radar_pc_data_save_to_file()
                self.radar_raw_adc_data_save_to_file()
            
            if self.lidar_enable:
                self.lidar_data_save_to_file()
            
            if self.camera_enable:
                self.camera_data_save_to_file()

            if self.vicon_enable:
                self.vicon_data_save_to_file()

            #handle imu and vehicle odometry measurements
            self.save_high_rate_sensor_buffers()

            #increment the sample index
            self.sample_idx += 1
        pass

    def update_high_rate_sensors(self):

        if self.check_latest_data():
            cur_time = self.get_clock().now()
            
            if self.imu_enable:
                self.imu_data_update_buffer(cur_time)
            if self.vehicle_odom_enable:
                self.vehicle_odometry_update_buffer(cur_time)        
        return
    
    ####################################################################
    # Checking / Processing data from each sensor
    ####################################################################
    def check_latest_data(self)->bool:
        """Check to make sure that all expected data is being streamed

        Returns:
            bool: True if all data is now being streamed, false if not
        """
        if self.radar_enable:

            for i in range(len(self.radar_pc_msgs_latest)):
                #check radar point clouds
                if self.radar_pc_msgs_latest[i] is None:
                    self.get_logger().info(
                        "No radar pc data from {}".format(self.radar_pc_topics[i]))
                    return False
            for i in range(len(self.radar_raw_adc_msgs_latest)):
                #check radar adc data
                if self.radar_raw_adc_msgs_latest[i] is None:
                    self.get_logger().info(
                        "No radar raw adc data from {}".format(self.radar_raw_adc_topics[i]))
                    return False
        if self.lidar_enable:
            if self.lidar_data_msg_latest is None:
                self.get_logger().info(
                    "No lidar data"
                )
                return False
        if self.camera_enable:
            if self.camera_msg_latest is None:
                self.get_logger().info(
                    "No camera data"
                )
                return False
        if self.imu_enable:
            if self.imu_data_msg_latest is None:
                self.get_logger().info(
                    "No imu data"
                )
                return False
        if self.vicon_enable:
            for i in range(len(self.vicon_msgs_latest)):
                #check radar adc data
                if self.vicon_msgs_latest[i] is None:
                    self.get_logger().info(
                        "No vicon data from {}".format(self.vicon_topics[i]))
                    return False
        if self.vehicle_odom_enable:
            if self.vehicle_odom_msg_latest is None:
                self.get_logger().info(
                    "No vehicle_odom data"
                )
                return False
        
        return True
    

    def clear_high_speed_sensor_buffers(self):

        if self.imu_enable:
            self.imu_data_buffer = []
        if self.vehicle_odom_enable:
            self.vehicle_odom_data_buffer = []
    
    def save_high_rate_sensor_buffers(self):
        """Sample the high rate data buffers at the same time
        and save their data to a file
        """
        #save the arrays at the same time
        if self.imu_enable:
            imu_data = np.array(self.imu_data_buffer)
        if self.vehicle_odom_enable:
            # if len(self.vehicle_odom_data_buffer) == 1:
            #     odom_data = np.array([self.vehicle_odom_data_buffer])
            # else:
            odom_data = np.array(self.vehicle_odom_data_buffer)
        
        self.get_logger().info("odom array shape: {}, len:{}".format(
            odom_data.shape,len(self.vehicle_odom_data_buffer)
        ))
        vel_data = odom_data[:,[0,8,13]]
        
        #reset the buffers so they continue capturing data
        self.clear_high_speed_sensor_buffers()

        #save the data to a file
        if self.imu_enable:
            self.imu_data_save_to_file(imu_data)
        if self.vehicle_odom_enable:
            self.vehicle_odom_data_save_to_file(odom_data)
            self.vehicle_vel_data_save_to_file(vel_data)

    def imu_data_update_buffer(self, time: Time):
        """Capture the latest imu data as [time,w_x,w_y,w_z,a_x,a_y,a_z]

        Args:
            time (Time): _description_
        """

        if self.imu_data_buffer is None:
            self.imu_data_buffer = []

        msg:Imu = self.imu_data_msg_latest

        # Prevent the imu_data_full buffer from growing too large
        if len(self.imu_data_buffer) > 100:
            self.imu_data_buffer = []
        else:
            self.imu_data_buffer.append([
                time.nanoseconds * 1e-9,  # Convert to seconds as float
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
    
    def vehicle_odometry_update_buffer(self,time:Time):
        """Capture the latest odometry data as 
            [time,x,y,z,quat_w,quat_x,quat_y,quat_z,vx,vy,vz,wx,wy,wz]

        Args:
            time (Time): _description_
        """
        if self.vehicle_odom_data_buffer is None:
            self.vehicle_odom_data_buffer = []
        
        #reset the buffer if it gets too big
        if len(self.vehicle_odom_data_buffer) > 100:
            self.vehicle_odom_data_buffer = []
        
        msg:Odometry = self.vehicle_odom_msg_latest
        self.vehicle_odom_data_buffer.append(
            [
                time.nanoseconds * 1e-9,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ]
        )

    ####################################################################
    # Saving sensor data to a file
    ####################################################################

    def radar_pc_data_save_to_file(self):

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)

        for i in range(len(self.radar_pc_folders)):

            folder = self.radar_pc_folders[i]
            msg:PointCloud2 = self.radar_pc_msgs_latest[i]

            #transform to base frame
            msg = self.transform_pc2_to_base_frame(msg)

            if msg:
                #convert to np array
                data = self.pointcloud2_to_np_radar(msg)

                path = os.path.join(self.dataset_path,folder,file_name)
                np.save(path,data)

                self.get_logger().info("Received {} detections from {}".format(
                    data.shape[0],
                    folder
                ))
            else:
                self.get_logger().info("Failed to save data from {}".format(
                    folder
                ))
    
    def radar_raw_adc_data_save_to_file(self):

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)

        for i in range(len(self.radar_raw_adc_folders)):

            folder = self.radar_raw_adc_folders[i]
            msg:ADCDataCube = self.radar_raw_adc_msgs_latest[i]

            #get the real data
            real_data = np.array(msg.real_data)
            real_data = real_data.reshape(
                (msg.layout.dim[0].size,
                msg.layout.dim[1].size,
                msg.layout.dim[2].size)
            )

            #get the imag data
            imag_data = np.array(msg.imag_data)
            imag_data = imag_data.reshape(
                (msg.layout.dim[0].size,
                msg.layout.dim[1].size,
                msg.layout.dim[2].size)
            )

            data = real_data + 1j * imag_data

            path = os.path.join(self.dataset_path,folder,file_name)
            np.save(path,data)

            # log receiving the array
            rx_channels = msg.layout.dim[0].size
            samples_per_chirp = msg.layout.dim[1].size
            chirps_per_frame = msg.layout.dim[2].size
            sent_time = msg.header.stamp
            sent_time_secs = sent_time.sec + sent_time.nanosec * 1e-9

            out_status = \
                "Received ADCDataCube from {}: rx channels: {}, samples: {}, chirps: {}, time: {}".format(
                    folder,rx_channels,samples_per_chirp,chirps_per_frame,sent_time_secs)

            self.get_logger().info(out_status)
    
    def lidar_data_save_to_file(self):
        """Save lidar point cloud data (automatically transformed to base_frame)
        stored with at least (x,y,z) information
        """
        #transform the message to the base frame
        msg:PointCloud2 = self.transform_pc2_to_base_frame(self.lidar_data_msg_latest)

        if msg:
            #convert to np array
            data:np.ndarray = self.pointcloud2_to_np_lidar(msg)

            file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
            path = os.path.join(self.dataset_path,self.lidar_data_folder,file_name)
            np.save(path,data)

            self.get_logger().info("Saved lidar data")
        else:
            self.get_logger().info("Failed to save data from lidar")
    
    def camera_data_save_to_file(self):
        
        msg:Image = self.camera_msg_latest

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg=msg,desired_encoding="bgr8")

        file_name = "{}_{}.png".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.camera_data_folder,file_name)

        cv2.imwrite(path,cv_image)
        self.get_logger().info("Saved camera data")
    
    
    def imu_data_save_to_file(self,data:np.ndarray):
        """Save the imu data to a file

        Args:
            data (np.ndarray): IMU data with rows defined by [time,w_x,w_y,w_z,a_x,a_y,a_z]
        """

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.imu_data_folder,file_name)
        np.save(path,data)

        self.get_logger().info("Saved IMU data")
        return
    
    def vicon_data_save_to_file(self):
        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)

        for i in range(len(self.vicon_topics)):
            folder = self.vicon_data_folders[i]
            msg:TransformStamped = self.vicon_msgs_latest[i]

            # Translation (in meters)
            t_x = msg.transform.translation.x
            t_y = msg.transform.translation.y
            t_z = msg.transform.translation.z

            # Rotation (as a quaternion)
            r_w = msg.transform.rotation.w
            r_x = msg.transform.rotation.x
            r_y = msg.transform.rotation.y
            r_z = msg.transform.rotation.z

            # Save the data
            data = np.array([t_x, t_y, t_z, r_w, r_x, r_y, r_z])
            path = os.path.join(self.dataset_path, folder, file_name)
            np.save(path, data)

            out_status = "{} had vicon pose of ({})m".format(
                self.vicon_topics[i].split("/")[-1],[t_x,t_y,t_z]
            )
            self.get_logger().info(out_status)

    def vehicle_vel_data_save_to_file(self,data:np.ndarray):
        """Save the vehicle velocity data to a file

        Args:
            data (np.ndarray): vehicle velocity data with rows defined by
              [time,vx,wz]
        """

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.vehicle_vel_folder,file_name)
        np.save(path,data)

        self.get_logger().info("Saved vehicle velocity data")

        return
    
    def vehicle_odom_data_save_to_file(self,data:np.ndarray):
        """Save the vehicle odometry data to a file

        Args:
            data (np.ndarray): vehicle odometry data with rows defined by
              [time,x,y,z,quat_w,quat_x,quat_y,quat_z,vx,vy,vz,wx,wy,wz]
        """

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.vehicle_odom_folder,file_name)
        np.save(path,data)

        self.get_logger().info("Saved vehicle odometry data")

        return
    ####################################################################
    # Helper functions for dealing with point clouds
    ####################################################################

    def transform_pc2_to_base_frame(self,msg:PointCloud2)->PointCloud2:
        """Transform a pointCloud2 into the base_frame

        Args:
            msg (PointCloud2): PoinCloud2 object to transform into the base link

        Returns:
            PointCloud2: a new PointCloud2 object in the base_frame
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            transformed_cloud = do_transform_cloud(
                cloud=msg,
                transform=transform
            )

            return transformed_cloud
        
        except Exception as e:
            self.get_logger().info("could not transform point cloud: {}".format(e))

            return None
    
    def pointcloud2_to_np(self,msg:PointCloud2)->np.ndarray:
        """Converts a PointCloud2 array into a numpy array

        Args:
            msg (PointCloud2): The pointCloud2 object to convert to a numpy array

        Returns:
            np.ndarray: Numpy array of points from the PointCloud2 array
        """
        point_cloud = pc2.read_points_numpy(
            cloud=msg,
            skip_nans=True,
            reshape_organized_cloud=True
        )

        return point_cloud
    
    def pointcloud2_to_np_radar(self,msg:PointCloud2)->np.ndarray:
        """Converts a PointCloud2 array  from a radar into a numpy array

        Args:
            msg (PointCloud2): The pointCloud2 object to convert to a numpy array

        Returns:
            np.ndarray: Numpy array of points from the PointCloud2 array
        """
        point_cloud = pc2.read_points_numpy(
            cloud=msg,
            skip_nans=True,
            field_names=['x','y','z','vel'],
            reshape_organized_cloud=True
        )

        return point_cloud
    
    def pointcloud2_to_np_lidar(self,msg:PointCloud2)->np.ndarray:
        """Converts a PointCloud2 array  from a radar into a numpy array

        Args:
            msg (PointCloud2): The pointCloud2 object to convert to a numpy array

        Returns:
            np.ndarray: Numpy array of points from the PointCloud2 array
        """
        point_cloud = pc2.read_points_numpy(
            cloud=msg,
            skip_nans=True,
            field_names=['x','y','z','intensity'],
            reshape_organized_cloud=True
        )

        return point_cloud


def main(args=None):
    rclpy.init(args=args)
    node = DatasetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
