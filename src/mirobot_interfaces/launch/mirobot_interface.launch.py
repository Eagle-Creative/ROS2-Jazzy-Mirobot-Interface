import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Paths from the mirobot_description package
    description_pkg_path = os.path.join(get_package_share_directory("mirobot_description"))
    rviz_config_file = os.path.join(description_pkg_path, "rviz", "description.rviz")
    urdf_file = os.path.join(description_pkg_path, "urdf", "mirobot_urdf_2.urdf")
    serial_config = os.path.join(description_pkg_path, "config", "rviz_params.yaml")

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    PathJoinSubstitution([
                        FindExecutable(name='xacro')
                    ]),
                    ' ',
                    urdf_file
                ]),
                value_type=str
            ),
            'use_sim_time': True
        }],
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Serial Node
    serial_node = Node(
        package="mirobot_interfaces",
        executable="mirobot_gcode_writer",
        name="mirobot_write_node",
        output="screen",
        arguments=["-d", serial_config],
    )

    # Mirobot Interface Node
    mirobot_interface_node = Node(
        package="mirobot_interfaces",
        executable="mirobot_interface.py",
        name="mirobot_interface",
        output="screen",
    )

    # RViz Node (delayed startup)
    rviz2 = TimerAction(
        period=2.0,  # Delay RViz startup by 2 seconds
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        #joint_state_publisher_gui,  # Add the joint_state_publisher_gui back
        serial_node,
        mirobot_interface_node,
        rviz2,  # Start RViz after a delay
    ])
