import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    tutorial_pkg_dir = get_package_share_directory('orb3_ros2_wrapper')
    ORBvoc_file = DeclareLaunchArgument(
        'ORBvoc_file',
        default_value=os.path.join(
            tutorial_pkg_dir, 'config/ORBvoc.txt'),
        description='Path to the ORB vocabulary file'
    )
    camera_params_file = DeclareLaunchArgument(
        'camera_params_file',
        default_value=os.path.join(
            tutorial_pkg_dir, 'config/camera_params.yaml'),
        description='camera_params_file'
    )
    enable_pangolin = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='true',
        description='Enable Pangolin visualization (true or false)'
    )

    image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/carla/ego_vehicle/rgb_front/image',
        description='Subscriber Image topic'
    )

    parent_frame = DeclareLaunchArgument(
        'parent_frame',
        default_value='ego_vehicle/rgb_front',
        description='world or parent frame_id'
    )

    camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera',
        description='child frame_id'
    )

    mono_node = Node(
        package="orb3_ros2_wrapper",
        executable="mono_node",
        name="mono_node",
        parameters=[
            {'ORBvoc_file': LaunchConfiguration("ORBvoc_file")},
            {'camera_params_file': LaunchConfiguration(
                "camera_params_file")},
            {'enable_pangolin': LaunchConfiguration('enable_pangolin')},
            {'image_topic': LaunchConfiguration('image_topic')},
            {'parent_frame': LaunchConfiguration('parent_frame')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
        ],
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(ORBvoc_file)
    ld.add_action(camera_params_file)
    ld.add_action(enable_pangolin)
    ld.add_action(image_topic)
    ld.add_action(parent_frame)
    ld.add_action(camera_frame)
    ld.add_action(mono_node)
    return ld
