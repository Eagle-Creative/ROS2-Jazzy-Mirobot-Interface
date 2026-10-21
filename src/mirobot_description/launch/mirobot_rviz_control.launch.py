import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="mirobot_01",
        description="Robot namespace for namespaced joint states"
    )

    pkg_path = os.path.join(get_package_share_directory("mirobot_description"))
    rviz_config_file = os.path.join(pkg_path, "rviz", "description.rviz")
    urdf_file = os.path.join(pkg_path, "urdf", "mirobot_urdf_2.urdf")
    serial_config = os.path.join(pkg_path, "config", "rviz_params.yaml")

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],  # wall clock: no /clock without a simulator
        arguments=[urdf_file],
        remappings=[('/joint_states', ['/', LaunchConfiguration('robot_name'), '/joint_states'])],
    )

    # Launch RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    serial_node = Node(
        package="mirobot_description",
        executable="mirobot_gcode_writer",
        name="mirobot_write_node",
        output="screen",
        arguments=["-d", serial_config],
    )

    return LaunchDescription([
        robot_name_arg,
        TimerAction(
            period=3.0,
            actions=[rviz2]
        ),
        robot_state_publisher,
        joint_state_publisher_gui,
        serial_node,
    ])