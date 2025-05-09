import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("mirobot_description"))
    rviz_config_file = os.path.join(pkg_path, "rviz", "description.rviz")
    urdf_file = os.path.join(pkg_path, "urdf", "mirobot_urdf_2.urdf")
    serial_config = os.path.join(pkg_path, "config", "isaac_params.yaml")

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
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
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
        executable="mirobot_gcode_writer_isaac",
        name="mirobot_write_node",
        output="screen",
        arguments=["-d", serial_config],
    )

    return LaunchDescription([
        serial_node
    ])