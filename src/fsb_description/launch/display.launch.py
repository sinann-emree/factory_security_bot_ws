"""
robot_state_publisher + (isteğe bağlı) joint_state_publisher_gui + RViz.

Kullanım:
  ros2 launch fsb_description display.launch.py
  ros2 launch fsb_description display.launch.py gui:=false rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('fsb_description')
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'fsb_robot.urdf.xacro']),
            ' use_sim:=', use_sim,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='URDF ros2_control eklentisi: true=Gazebo, false=gerçek donanım'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='/clock zamanı kullan (simülasyonda true)'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true: joint_state_publisher_gui, false: joint_state_publisher'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz başlat'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'fsb_description.rviz'])],
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
