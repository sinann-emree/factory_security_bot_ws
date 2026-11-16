"""
Kapsamlı devriye: patrol_executor (+ isteğe bağlı coverage_monitor).

Nav2 çalışırken (bringup localization:=amcl):
  ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true [mode:=idleness] [loops:=5]
      [waypoints_file:=...] [priority_file:=...] [coverage:=true]
      [alert_hold:=5.0] [alert_cooldown:=120.0]   # /security/alerts tepkisi (sim-s)
Varsayılan waypoint dosyası: config/waypoints_factory.yaml (patrol_planner çıktısı).
Eski 4 noktalı rota (fallback): ros2 launch fsb_patrol patrol.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cfg = PathJoinSubstitution([FindPackageShare('fsb_patrol'), 'config'])
    ust = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='idleness',
                              choices=['idleness', 'sequential', 'random']),
        DeclareLaunchArgument('loops', default_value='0', description='0 = sonsuz'),
        DeclareLaunchArgument('waypoints_file',
                              default_value=PathJoinSubstitution([cfg, 'waypoints_factory.yaml'])),
        DeclareLaunchArgument('priority_file',
                              default_value=PathJoinSubstitution(
                                  [cfg, 'priority_points_factory.yaml']),
                              description="dünyaya özel öncelikli noktalar; '' = yok"),
        DeclareLaunchArgument('coverage', default_value='false',
                              description='true: coverage_monitor da başlar'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [FindPackageShare('fsb_navigation'), 'maps', 'factory.yaml'])),
        DeclareLaunchArgument('coverage_png', default_value=''),
        DeclareLaunchArgument('alert_hold', default_value='5.0',
                              description='alarmda bekleme süresi (sim-s)'),
        DeclareLaunchArgument('alert_cooldown', default_value='120.0',
                              description='olay sonrası alarmların yok sayıldığı süre (sim-s)'),
        Node(package='fsb_patrol', executable='patrol_executor', name='fsb_patrol_executor',
             output='screen',
             parameters=[{'use_sim_time': ust,
                          'mode': LaunchConfiguration('mode'),
                          'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
                          'waypoints_file': LaunchConfiguration('waypoints_file'),
                          'priority_file': LaunchConfiguration('priority_file'),
                          'alert_hold': ParameterValue(LaunchConfiguration('alert_hold'),
                                                       value_type=float),
                          'alert_cooldown': ParameterValue(
                              LaunchConfiguration('alert_cooldown'), value_type=float)}]),
        Node(package='fsb_patrol', executable='coverage_monitor', name='fsb_coverage_monitor',
             output='screen', condition=IfCondition(LaunchConfiguration('coverage')),
             parameters=[{'use_sim_time': ust,
                          'map_yaml': LaunchConfiguration('map'),
                          'robot_params': PathJoinSubstitution([cfg, 'robot_params.yaml']),
                          'png': LaunchConfiguration('coverage_png')}]),
    ])
