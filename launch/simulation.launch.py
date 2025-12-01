import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition


def load_robot_description(context):
    config_value = LaunchConfiguration("config").perform(context)
    use_sim_time_val = LaunchConfiguration("use_sim_time").perform(context)
    use_real_inertia_val = LaunchConfiguration("use_real_inertia").perform(context)

    use_sim_time_bool = use_sim_time_val.lower() == "true"

    pkg_path = get_package_share_directory("robile_description")
    xacro_file = os.path.join(pkg_path, "robots", f"{config_value}.urdf.xacro")

    robot_description_config = xacro.process_file(
        xacro_file, mappings={"use_real_inertia": use_real_inertia_val}
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description_config.toxml()},
                {"use_sim_time": use_sim_time_bool},
            ],
        )
    ]


def generate_launch_description():
    declare_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config", default_value="4_wheel_config"),
        DeclareLaunchArgument("platform_max_lin_vel", default_value="1.0"),
        DeclareLaunchArgument("platform_max_ang_vel", default_value="1.0"),
        DeclareLaunchArgument("use_real_inertia", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("init_pos_x", default_value="0.0"),
        DeclareLaunchArgument("init_pos_y", default_value="0.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("robile_description"),
                "worlds",
                "simple_world.world",
            ),
        ),
    ]

    rviz_enabled = LaunchConfiguration("start_rviz")
    init_pos_x = LaunchConfiguration("init_pos_x")
    init_pos_y = LaunchConfiguration("init_pos_y")
    init_yaw = LaunchConfiguration("init_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world")

    pkg_path = get_package_share_directory("robile_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    ros_clock_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    odom_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/robile/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        ],
        remappings=[("/model/robile/odometry", "/odom")],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file],
        }.items(),
    )

    robot_state_publisher = OpaqueFunction(function=load_robot_description)

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robile", 
            "-x", init_pos_x,
            "-y", init_pos_y,
            "-z", "0.0",
            "-Y", init_yaw,
            "-topic", "/robot_description",
        ],
        output="screen",
    )

    platform_controller_node = Node(
        package="robile_gazebo",
        executable="robile_gazebo_platform_controller",
        output="screen",
        parameters=[
            {
                "platform_max_lin_vel": LaunchConfiguration("platform_max_lin_vel"),
                "platform_max_ang_vel": LaunchConfiguration("platform_max_ang_vel"),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    controller_processes = [
        ExecuteProcess(
            cmd=[
                "ros2",
                "control",
                "load_controller",
                "--set-state",
                "active",
                "robile_hub_wheels_controller",
            ],
            output="screen",
        )
    ]

    load_controllers_event_handler = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=controller_processes)
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("robile_gazebo"),
        "config",
        "rviz",
        "simulation.rviz",
    )
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(rviz_enabled),
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    return LaunchDescription(
        [
            *declare_arguments,
            ros_clock_node,
            gazebo,
            robot_state_publisher,
            spawn,
            load_joint_state_controller,
            load_controllers_event_handler,
            odom_bridge_node,
            platform_controller_node,
            rviz_node,
        ]
    )
