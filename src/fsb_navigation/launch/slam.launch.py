"""
SLAM Toolbox (online async, haritalama) + lifecycle_manager.

map→odom'u bu modda YALNIZCA slam_toolbox yayınlar (AMCL çalıştırılmaz).
Kullanım (genelde bringup üzerinden: localization:=slam):
  ros2 launch fsb_navigation slam.launch.py use_sim_time:=true
Haritayı kaydet:
  ros2 run nav2_map_server map_saver_cli -f <yol>/factory --ros-args -p use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'config', 'slam_toolbox_mapping.yaml'])),

        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time,
                                 'use_lifecycle_manager': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['slam_toolbox'],
                         'bond_timeout': 0.0}],
        ),
    ])
