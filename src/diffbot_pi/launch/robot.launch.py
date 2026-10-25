import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("diffbot_pi")
    cfg = os.path.join(share, "config", "robot.yaml")
    urdf = os.path.join(share, "urdf", "robot.urdf")
    slam_cfg = os.path.join(share, "config", "slam.yaml")

    slam_share = get_package_share_directory("slam_toolbox")
    nav_share = get_package_share_directory("nav2_bringup")

    with open(urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("nav2", default_value="false"),
        DeclareLaunchArgument("sonar", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("map", default_value=""),

        Node(
            package="diffbot_pi",
            executable="base_controller",
            name="base_controller",
            parameters=[cfg],
            output="screen",
        ),

        Node(
            package="diffbot_pi",
            executable="sonar",
            name="sonar",
            parameters=[cfg],
            condition=IfCondition(LaunchConfiguration("sonar")),
            output="screen",
        ),

        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar",
            parameters=[cfg],
            condition=IfCondition(LaunchConfiguration("lidar")),
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={"slam_params_file": slam_cfg}.items(),
            condition=IfCondition(LaunchConfiguration("slam")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, "launch", "navigation_launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("nav2")),
        ),
    ])
