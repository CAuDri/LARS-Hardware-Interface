from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = Path(get_package_share_directory("lars_description"))
    urdf_path = description_dir / "urdf" / "lars_dummy.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="lars_dummy_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 20.0,
                    }
                ],
            ),
        ]
    )
