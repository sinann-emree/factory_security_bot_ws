"""
Lokalizasyon (map_server + AMCL) ve Nav2 navigasyon yığını.

nav2_bringup'ın navigation_launch.py'si opennav_docking'i de başlattığı için
yalnızca gereken düğümler burada açıkça tanımlanır.

cmd_vel zinciri (TwistStamped):
  controller_server, behavior_server --cmd_vel_nav--> velocity_smoother
  --cmd_vel_smoothed--> collision_monitor --/cmd_vel--> diff_drive_controller
map→odom: YALNIZCA AMCL (SLAM bu modda çalışmaz).

Kullanım (genelde bringup üzerinden: localization:=amcl):
  ros2 launch fsb_navigation navigation.launch.py use_sim_time:=true map:=<map.yaml>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    common = [params, {'use_sim_time': use_sim_time}]
    nav_cmd = [('cmd_vel', 'cmd_vel_nav')]

    def nav_node(package: str, executable: str, name: str, remappings=None) -> Node:
        return Node(package=package, executable=executable, name=name, output='screen',
                    parameters=common, remappings=remappings or [])

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = ['controller_server', 'planner_server', 'behavior_server',
                        'bt_navigator', 'waypoint_follower', 'velocity_smoother',
                        'collision_monitor']

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'maps', 'factory.yaml'])),

        # --- Lokalizasyon ---
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]),
        nav_node('nav2_amcl', 'amcl', 'amcl'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': localization_nodes}]),

        # --- Navigasyon ---
        nav_node('nav2_controller', 'controller_server', 'controller_server', nav_cmd),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server', nav_cmd),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', nav_cmd),
        nav_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': navigation_nodes}]),
    ])
