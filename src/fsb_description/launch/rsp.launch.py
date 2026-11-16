"""
robot_state_publisher (GUI'siz). bringup.launch.py tarafından kullanılır.

Kullanım:
  ros2 launch fsb_description rsp.launch.py use_sim:=true use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('fsb_description'), 'urdf', 'fsb_robot.urdf.xacro']),
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
