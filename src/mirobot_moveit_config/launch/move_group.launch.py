# SPDX-License-Identifier: Apache-2.0
"""move_group for the WLKATA Mirobot (planning only).

Starts robot_state_publisher (model incl. mirobot_tcp) and move_group, both fed by the
Mirobot driver's /<robot_name>/joint_states.  Trajectory execution is off: the Mirobot has no
ros2_control interface; send planned joint targets through the driver instead (see README).

    ros2 launch mirobot_moveit_config move_group.launch.py tool_length:=0.0365
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _nodes(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    tool_length = LaunchConfiguration('tool_length').perform(context)
    cfg = (
        MoveItConfigsBuilder('mirobot', package_name='mirobot_moveit_config')
        .robot_description(file_path='urdf/mirobot_moveit.urdf.xacro', mappings={'tool_length': tool_length})
        .robot_description_semantic(file_path='srdf/mirobot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .planning_pipelines(pipelines=['pilz_industrial_motion_planner', 'ompl'],
                            default_planning_pipeline='pilz_industrial_motion_planner')
        .to_moveit_configs()
    )
    js = [('joint_states', f'/{robot_name}/joint_states')]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[cfg.robot_description, {'use_sim_time': False}], remappings=js),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[cfg.to_dict(), {'use_sim_time': False,
                                         'publish_robot_description_semantic': True,
                                         # plan-only: no controllers are managed here
                                         'moveit_manage_controllers': False,
                                         'allow_trajectory_execution': False}],
             remappings=js),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='mirobot_01'),
        DeclareLaunchArgument('tool_length', default_value='0.0365',
                              description='Firmware TCP (J6 flange face) to tool tip (m); '
                                          'default: WLKATA suction cup assembly'),
        OpaqueFunction(function=_nodes),
    ])
