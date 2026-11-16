"""
Gazebo'suz Nav2 planlayıcı arka ucu: patrol_planner --cost nav2 için gerçek Nav2 yol uzunlukları.

map_server + planner_server (+ global_costmap) fsb_navigation/config/nav2_params.yaml ile, yani
devriyede kullanılan AYNI planlayıcı ve costmap. Robot pozu gerekmediğinden (ComputePathToPose
use_start=true) map→odom→base_footprint birim statik TF ile verilir.
Duvar saati (use_sim_time=false).

Kullanım:
  ros2 launch fsb_patrol planner_backend.launch.py map:=<harita.yaml>
  ros2 run fsb_patrol patrol_planner --cost nav2 --map <harita.yaml> --out <waypoints.yaml>
Not: Simülasyon (bringup localization:=amcl) zaten çalışıyorsa bu launch'a gerek yoktur;
     planlayıcı aynı action'ı (compute_path_to_pose) çalışan Nav2'den kullanır.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration('params_file')
    common = {'use_sim_time': False}
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'maps', 'factory.yaml'])),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'config', 'nav2_params.yaml'])),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='odom_to_base',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint']),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, common, {'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner_backend', output='screen',
             parameters=[common, {'autostart': True,
                                  'node_names': ['map_server', 'planner_server']}]),
    ])
