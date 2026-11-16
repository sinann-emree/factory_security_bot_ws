"""
Döngüsel devriye (Nav2 FollowWaypoints).

Nav2 (bringup localization:=amcl) çalışırken:
  ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true [loops:=10]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('loops', default_value='0', description='0 = sonsuz'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_patrol'), 'config', 'patrol.yaml'])),
        Node(
            package='fsb_patrol', executable='patrol_node', name='fsb_patrol', output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int)}],
        ),
    ])
