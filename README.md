# orb3_ros2_wrapper

ROS2 package that provides a wrapper for the ORB-SLAM3 monocular node without IMU.

## Overview

The `orb3_ros2_wrapper` package wraps the monocular node of ORB-SLAM3, a popular visual SLAM library, to provide visual odometry and mapping capabilities in ROS2 without using IMU data. It enables the use of a monocular camera as the sole sensor for estimating camera pose and creating a 3D map of the environment.

## Dependencies

- ORB-SLAM3: You need to install ORB-SLAM3 library and its dependencies. Follow the installation instructions provided by ORB-SLAM3.

## Build Instructions

1. Clone this repository to your ROS2 workspace.
2. To update the ORB-SLAM3 installation directory, follow these steps:
    1. Open the `CMakeLists.txt` file located in the root directory of this ROS2 package.
    2. Locate the following lines of code:

        ```cmake

        set(ORB_SLAM3_DIR
        $ENV{HOME}/ORB_SLAM3)
        ```

    3. Modify the `ORB_SLAM3_DIR` variable to point to the correct directory where you have installed ORB-SLAM3. For    example:

        ```cmake
        set(ORB_SLAM3_DIR
        /path/to/ORB_SLAM3
        )
        ```

    4. Save the `CMakeLists.txt` file.
3. Build the package using colcon or a similar ROS2 build tool:

```bash
colcon build --packages-select orb3_ros2_wrapper --symlink-install
```

## Config Directory

The orb3_ros2_wrapper package should have a config directory which contains the following files:

- `ORBvoc.txt`: The ORB vocabulary file used by ORB-SLAM3. Copy the ORBvoc.txt file from ORB-SLAM3/Vocabulary/ folder to the config folder in this package.
- `camera_params.yaml`: The camera parameters file specifying the intrinsic calibration parameters for the camera.

Make sure that you have these files in the `config` directory of your `orb3_ros2_wrapper` package before running the launch file with default arguments.

## Nodes

`mono_node`: Executable from the orb3_ros2_wrapper package, which performs visual SLAM using ORB-SLAM3. It is launched with the specified parameters and arguments. This node also broadcast tf between  `parent_frame` `camera_frame`

## Topics

`image_topic`: The input image topic to subscribe to for visual data.
`orb3/camera/pose` :  output topic, camera pose in after transforming with extrinsic parameter

## Parameters

- `ORBvoc_file`: Path to the ORB vocabulary file.
- `camera_params_file`: Path to the camera parameters file.
- `enable_pangolin`: Enable or disable Pangolin visualization.
- `image_topic`: Image topic to subscribe to.
- `parent_frame`: Frame ID of the parent frame.
- `camera_frame`: Frame ID of the camera.

## Usage

```bash
ros2 launch orb3_ros2_wrapper orb3_ros2_wrapper.launch.py
```

## Launch Arguments

The following launch arguments can be passed to the `orb3_ros2_wrapper.launch.py` file:

- `ORBvoc_file` (string, default: `/path/to/your/ros2_ws/src/orb3_ros2_wrapper/config/ORBvoc.txt`): Path to the ORB vocabulary file.
- `camera_params_file` (string, default: `/path/to/your/ros2_ws/src/orb3_ros2_wrapper/config/camera_params.yaml`): Path to the camera parameters file.
- `enable_pangolin` (boolean, default: `true`): Enable or disable Pangolin visualization (true or false).
- `image_topic` (string, default: `/carla/ego_vehicle/rgb_front/image`): Image topic to subscribe to.
- `parent_frame` (string, default: `ego_vehicle/rgb_front`): Frame ID of the parent frame.
- `camera_frame` (string, default: `camera`): Frame ID of the camera.

You can modify these launch arguments to suit your specific setup.

## Demo with CARLA Simulator

1. Build and install carla ros2 bridge, refer to the [ROS bridge installation for ROS2](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2)
2. Run carla ros2 bridge with example:

    ```bash
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py town:=Town03
    ```

3. Run `mono_node` from launch file

    ```bash
    ros2 launch orb3_ros2_wrapper orb3_monocular.launch.py image_topic:=/carla/ego_vehicle/rgb_front/image
    ```

4. you need transform `/orb3/camera/pose` to desired frame say `map` frame and publish it on topic say ```/orb3/camera/transformed/pose```

5. `ros2 topic list | grep orb` , and you can use plotjuggler to plot `/carla/ego_vehicle/odometry` and `/orb3/camera/transformed/pose`

![alt text](https://github.com/Basavaraj-PN/orb3_ros2_wrapper/blob/main/resources/screenshots/demo.png)

For more information on how to use ORB3 ROS2 Wrapper, refer to the [ORB3 ROS2 Wrapper GitHub repository](https://github.com/Basavaraj-PN/orb3_ros2_wrapper).
