# CPSL Datasets
Package for accessing various CPSL datasets

## Installation
In order for the code to work properly, the following steps are required
1. Install correct version of python
2. Install cpsl_datasets using Poetry

### 1. Setup Python environment

#### Deadsnakes PPA (requires sudo access)
1. On ubuntu systems, start by adding the deadsnakes PPA to add the required version of python.
```
sudo add-apt-repository ppa:deadsnakes/ppa
```

2. Update the package list
```
sudo apt update
```

3. Install python 3.10 along with the required development dependencies
```
sudo apt install python3.10 python3.10-dev
```

The following resources may be helpful [Deadsnakes PPA description](https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa), [Tutorial on Deadsnakes on Ubuntu](https://preocts.github.io/python/20221230-deadsnakes/)

#### Conda (Backup)
1. If conda isn't already installed, follow the [Conda Install Instructions](https://conda.io/projects/conda/en/stable/user-guide/install/index.html) to install conda
2. Use the following command to download the conda installation (for linux)
```
wget https://repo.anaconda.com/archive/Anaconda3-2023.09-0-Linux-x86_64.sh
```
3. Run the conda installation script (-b for auto accepting the license)
```
bash Anaconda3-2023.09-0-Linux-x86_64.sh -b
```
3. Once conda is installed, create a new conda environment with the correct version of python
```
conda create -n cpsl_datasets python=3.10
```

### 2. Clone cpsl_datasets
```
git clone https://github.com/davidmhunt/cpsl_datasets.git
```
Initialize the submodules
```
cd cpsl_datasets
git submodule update --init
```
### 3. Install cpsl_datasets using Poetry

#### Installing Poetry:
 
1. Check to see if Python Poetry is installed. If the below command is successful, poetry is installed move on to setting up the conda environment

```
    #for now ensure that it is lower than version 2.0 (TOML files are out of date)
    poetry --version
```
2. If Python Poetry is not installed, follow the [Poetry Install Instructions](https://python-poetry.org/docs/#installing-with-the-official-installer). On linux, Poetry can be installed using the following command:
```
curl -sSL https://install.python-poetry.org | python3 - --version 1.8.4
```

If you are using poetry over an ssh connection or get an error in the following steps, try running the following command first and then continuing with the remainder fo the installation.
```
export PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
```
### Installing cpsl_datasets
If your machine supports it Navigate to the cpsl_datasets foler (this folder) and execute the following command

```
poetry install --extras "submodules"
```

### Using .env for Project Directories

In order to use any datasets in your computer's directory, you must first create a .env file and mark where the dataset files can be found.

1. Create a .env file in your project's root directory. This will file will not be uploaded to GitHub when you commit your changes.
2. Inside the .env file, add these variables
```
CPSL_DATASET_DIRECTORY=/data/radnav/rad_nav_datasets/
MAP_DIRECTORY=/data/radnav/maps/
AHENA_DATASET_DIRECTORY=/data/athena_uav_demo
```
3. Replace the example text with the path to your directory


## Accessing Datasets with CpslDS

The `CpslDS` class provides a simple interface to access the various sensor data in the CPSL datasets.

### Initialization

To use the class, first import it and create an instance by providing the path to the dataset:

```python
from cpsl_datasets import CpslDS
import os

#get the dataset path from the environment variable
dataset_path = os.environ.get("MY_DATASET_PATH")

#initialize the CpslDS class
cpsl_ds = CpslDS(dataset_path)
```

When initializing, you can also specify the names of the folders for each sensor if they are different from the default values. The following arguments can be provided to the `CpslDS` constructor:

> **Note:** `radar_folder` is now legacy and will default to an empty string. Prefer `radar_adc_folder` and `radar_pc_folder` for the raw ADC cubes and processed point clouds respectively.

| Argument | Description | Default Value |
|---|---|---|
| `dataset_path` | The path to the dataset directory. | (required) |
| `radar_folder` | The name of the folder containing the radar data. | `""` (deprecated when ADC/point-cloud separates are present) |
| `radar_adc_folder` | The folder containing the raw radar ADC cubes. | `"radar_adc"` |
| `radar_pc_folder` | The folder containing the radar point cloud outputs. | `"radar_pc"` |
| `lidar_folder` | The name of the folder containing the lidar data. | `"lidar"` |
| `camera_folder` | The name of the folder containing the camera data. | `"camera"` |
| `hand_tracking_folder` | The name of the folder containing the hand tracking data. | `"hand_tracking"` |
| `leap_motion_image_left_folder` | The name of the folder containing the left Leap Motion camera images. | `"leap_images/left"` |
| `leap_motion_image_right_folder` | The name of the folder containing the right Leap Motion camera images. | `"leap_images/right"` |
| `imu_orientation_folder` | The name of the folder containing the IMU orientation data. | `"imu_data"` |
| `imu_full_folder` | The name of the folder containing the full IMU data. | `"imu_full"` |
| `vehicle_vel_folder` | The name of the folder containing the vehicle velocity data. | `"vehicle_vel"` |
| `vehicle_odom_folder` | The name of the folder containing the vehicle odometry data. | `"vehicle_odom"` |

### Accessing Sensor Data

Once the `CpslDS` object is created, you can access the data for each sensor using the following methods. Each method takes an index `idx` which corresponds to the frame number in the dataset.

-   **Radar Data (new APIs)**:
    - `get_radar_adc_data(idx)`: Returns the raw radar ADC cube as a complex numpy array of shape `(rx_channels, samples, chirps)`.
    - `get_radar_point_cloud(idx)`: Returns the radar point cloud as a `(N, 4)` numpy array of `[x, y, z, velocity]` in the base frame.
    - **Deprecated** `get_radar_data(idx)`: Kept for backward compatibility but will be removed soon. Please migrate callers to either `get_radar_adc_data` or `get_radar_point_cloud` depending on the data you need.

-   **Lidar Data**:
    -   `get_lidar_point_cloud(idx)`: Returns a filtered `(N, 2)` numpy array of lidar detections `[x, y]`.
    -   `get_lidar_point_cloud_raw(idx)`: Returns a raw `(N, 4)` numpy array of lidar detections `[x, y, z, intensity]`.

-   **Camera Data**:
    `get_camera_frame(idx)`: Returns a numpy array with RGB channels representing the camera image.

-   **Hand Tracking Data**:
    `get_hand_tracking_data(idx)`: Returns a `(21, 3)` numpy array of hand joint positions `[x, y, z]`. The joint order is: Palm, Thumb (4 joints), Index (4), Middle (4), Ring (4), Pinky (4).

-   **Leap Motion Image Data**:
    -   `get_leap_motion_image_frame_left(idx)`: Returns a numpy array representing the left leap motion camera image.
    -   `get_leap_motion_image_frame_right(idx)`: Returns a numpy array representing the right leap motion camera image.

-   **IMU Data**:
    -   `get_imu_orientation_rad(idx)`: Returns the IMU heading in radians (`float` from -pi to pi).
    -   `get_imu_full_data(idx)`: Returns a `(N, 7)` numpy array with the full IMU data `[time, w_x, w_y, w_z, acc_x, acc_y, acc_z]`.

-   **Vehicle Data**:
    -   `get_vehicle_vel_data(idx)`: Returns a `(N, 3)` numpy array with vehicle velocity data `[time, vx, wz]`.
    -   `get_vehicle_odom_data(idx)`: Returns a `(N, 14)` numpy array with the full vehicle odometry data `[time, x, y, z, quat_w, quat_x, quat_y, quat_z, vx, vy, vz, wx, wy, wz]`.
        - Note that velocity data is in the vehicle's coordinate frame, not the global coordinate frame.

You can get the total number of frames in the dataset using the `num_frames` attribute:

```python
num_frames = cpsl_ds.num_frames
print(f"The dataset has {num_frames} frames.")

#loop through the dataset and get the data for each frame
for i in range(num_frames):
    #get the radar data for the i-th frame
    radar_data = cpsl_ds.get_radar_data(i)

    #get the lidar data for the i-th frame
    lidar_data = cpsl_ds.get_lidar_point_cloud(i)

    #... and so on for the other sensors
```

### Coordinate Frame Notes

Generally, all data should be stored in the standard ROS2 FLU (+X forward, +Y left, +Z up) coordinate frame. 
