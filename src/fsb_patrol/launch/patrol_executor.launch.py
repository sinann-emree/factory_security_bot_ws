"""
Kapsamlı devriye: patrol_executor (+ isteğe bağlı coverage_monitor).

Nav2 çalışırken (bringup localization:=amcl):
  ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true [mode:=idleness] [loops:=5]
      [waypoints_file:=...] [priority_file:=...] [coverage:=true]
      [revisit_interval:=300.0] [inspect_views:=3] [merge_radius:=1.0]
      [alert_log_dir:=~/.ros/fsb_security]   # yatan insan olay listesi (incident_manager.py)
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
        DeclareLaunchArgument('revisit_interval', default_value='300.0',
                              description='yatan insan kaydından sonra revisit süresi (sim-s)'),
        DeclareLaunchArgument('inspect_views', default_value='3',
                              description='kayıt için farklı açı sayısı'),
        DeclareLaunchArgument('merge_radius', default_value='1.0',
                              description='aynı kişi sayılan tespitlerin yarıçapı (m)'),
        DeclareLaunchArgument('alert_log_dir', default_value='~/.ros/fsb_security',
                              description='watchlist.yaml, events.jsonl ve kareler'),
        Node(package='fsb_patrol', executable='patrol_executor', name='fsb_patrol_executor',
             output='screen',
             parameters=[{'use_sim_time': ust,
                          'mode': LaunchConfiguration('mode'),
                          'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
                          'waypoints_file': LaunchConfiguration('waypoints_file'),
                          'priority_file': LaunchConfiguration('priority_file'),
                          'revisit_interval': ParameterValue(
                              LaunchConfiguration('revisit_interval'), value_type=float),
                          'inspect_views': ParameterValue(
                              LaunchConfiguration('inspect_views'), value_type=int),
                          'merge_radius': ParameterValue(
                              LaunchConfiguration('merge_radius'), value_type=float),
                          'alert_log_dir': LaunchConfiguration('alert_log_dir')}]),
        Node(package='fsb_patrol', executable='coverage_monitor', name='fsb_coverage_monitor',
             output='screen', condition=IfCondition(LaunchConfiguration('coverage')),
             parameters=[{'use_sim_time': ust,
                          'map_yaml': LaunchConfiguration('map'),
                          'robot_params': PathJoinSubstitution([cfg, 'robot_params.yaml']),
                          'png': LaunchConfiguration('coverage_png')}]),
    ])
