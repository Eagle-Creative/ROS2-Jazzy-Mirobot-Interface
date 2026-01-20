import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('mirobot_interfaces'),
            'config',
            'mirobot_config.yaml'
        ]),
        description='Path to the Mirobot configuration file'
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    # Get launch configuration values
    config_file = LaunchConfiguration('config_file')
    enable_rviz = LaunchConfiguration('enable_rviz')

    # Paths from the mirobot_description package
    description_pkg_path = get_package_share_directory("mirobot_description")
    rviz_config_file = os.path.join(description_pkg_path, "rviz", "description.rviz")
    urdf_file = os.path.join(description_pkg_path, "urdf", "mirobot_urdf_2.urdf")

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

    # G-code Writer Node (Serial Communication)
    gcode_writer_node = Node(
        package="mirobot_interfaces",
        executable="mirobot_gcode_writer",
        name="mirobot_gcode_writer",
        output="screen",
        parameters=[config_file],
    )

    # Mirobot Standard Functions Node
    mirobot_std_functions_node = Node(
        package="mirobot_interfaces",
        executable="mirobot_std_functions.py",
        name="mirobot_std_functions",
        output="screen",
        parameters=[config_file],
    )

    # Mirobot Controller Node
    mirobot_controller_node = Node(
        package="mirobot_interfaces",
        executable="mirobot_controller.py",
        name="mirobot_controller",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        config_file_arg,
        enable_rviz_arg,
        robot_state_publisher,
        gcode_writer_node,
        mirobot_std_functions_node,
        mirobot_controller_node,
    ])
