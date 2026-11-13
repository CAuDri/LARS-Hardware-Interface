from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_launch = (
        Path(get_package_share_directory("lars_description"))
        / "launch" / "robot_state_publisher.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description_launch))),
            Node(
                package="lars_visualizer",
                executable="steering_joint_state_bridge.py",
                name="lars_steering_joint_state_bridge",
                output="screen",
            ),
            Node(
                package="lars_visualizer",
                executable="imu_orientation_tf_bridge.py",
                name="lars_imu_orientation_tf_bridge",
                output="screen",
            ),
        ]
    )
