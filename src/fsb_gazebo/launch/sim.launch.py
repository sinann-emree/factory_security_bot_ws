"""
Gazebo Harmonic: dünya + robot spawn + ros_gz_bridge.

robot_state_publisher'ın /robot_description yayınladığını varsayar (bringup.launch.py başlatır).
Kullanım (genelde bringup üzerinden):
  ros2 launch fsb_gazebo sim.launch.py headless:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('fsb_gazebo')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    def gz(extra_args: str, condition) -> IncludeLaunchDescription:
        # -r: başlar başlamaz çalıştır; -v 3: uyarı seviyesi log
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': ['-r -v 3 ', extra_args, world],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=condition,
        )

    gz_gui = gz('', UnlessCondition(headless))
    gz_headless = gz('-s ', IfCondition(headless))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=PathJoinSubstitution([pkg, 'worlds', 'factory.sdf']),
            description='SDF dünya dosyası'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: yalnızca sunucu (gz sim -s), GUI yok'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.01',
                              description='Hafif yukarıdan bırak (temas oturması)'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        gz_gui,
        gz_headless,

        # /clock köprüsü — tek /clock kaynağı
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{
                'config_file': PathJoinSubstitution([pkg, 'config', 'gz_bridge.yaml']),
                'use_sim_time': True,
            }],
        ),

        # Robotu /robot_description topic'inden spawn et
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_fsb_robot',
            output='screen',
            arguments=[
                '-world', 'factory',
                '-topic', 'robot_description',
                '-name', 'fsb_robot',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
            parameters=[{'use_sim_time': True}],
        ),
    ])
