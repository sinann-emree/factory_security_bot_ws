"""
Tek giriş noktası: simülasyon veya gerçek robot.

Kullanım:
  ros2 launch fsb_bringup bringup.launch.py                 # use_sim:=true, Gazebo GUI
  ros2 launch fsb_bringup bringup.launch.py headless:=true  # yalnızca gz sunucusu
  ros2 launch fsb_bringup bringup.launch.py use_sim:=false  # gerçek robot (Aşama 7)

Başlatma sırası:
  robot_state_publisher → (sim) Gazebo + bridge + spawn → joint_state_broadcaster spawner
  → (spawner bitince, OnProcessExit) diff_drive_controller spawner
use_sim_time = use_sim (tek noktadan, tasarım kuralı).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = use_sim  # simülasyonda true, gerçek robotta false
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('fsb_control'), 'config', 'controllers.yaml'])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_description'), 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim': use_sim, 'use_sim_time': use_sim_time}.items(),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_gazebo'), 'launch', 'sim.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
        condition=IfCondition(use_sim),
    )

    real_placeholder = LogInfo(
        msg='use_sim:=false — gerçek donanım (ros2_control_node + fsb_hardware) '
            "Aşama 7'de eklenecek; controller spawner'lar controller_manager bekleyecek.",
        condition=UnlessCondition(use_sim),
    )

    # controller_manager (sim: gz_ros2_control eklentisi, robot spawn edilince) hazır olana dek
    # spawner servis bekler — sabit TimerAction gecikmesine bağımlı değil.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_diff_drive_controller',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120',
                   '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='true: Gazebo simülasyonu, false: gerçek robot'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_gazebo'), 'worlds', 'factory.sdf']),
            description='Gazebo dünya dosyası (use_sim:=true)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: Gazebo GUI açılmaz (gz sim -s)'),

        rsp,
        sim,
        real_placeholder,
        jsb_spawner,
        # Sıra: broadcaster aktif olduktan sonra diff_drive (Skill 4 adım 4)
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[ddc_spawner])),
    ])
