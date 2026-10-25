import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("diffbot_pi")
    cfg = os.path.join(share, "config", "robot.yaml")
    urdf = os.path.join(share, "urdf", "robot.urdf")

    with open(urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar",
            parameters=[cfg],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
    ])
