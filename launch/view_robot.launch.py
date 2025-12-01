import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def load_robot_description(context):
    config_value = LaunchConfiguration("config").perform(context)
    pkg_path = get_package_share_directory("robile_description")
    xacro_file = os.path.join(pkg_path, "robots", f"{config_value}.urdf.xacro")

    # Process xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    ]


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    config = LaunchConfiguration("config")

    # RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory("robile_gazebo"), "config", "rviz", "robile.rviz"
    )

    # Declare launch arguments
    declare_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable joint_state_publisher_gui",
        ),
        DeclareLaunchArgument(
            "config",
            default_value="4_wheel_config",
            description="URDF xacro filename (without extension) to load",
        ),
    ]

    # Joint state publisher nodes
    joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz node with a small delay to ensure robot description is loaded
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription(
        [
            *declare_arguments,
            OpaqueFunction(function=load_robot_description),
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
