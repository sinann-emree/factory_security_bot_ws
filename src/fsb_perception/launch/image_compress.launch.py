"""
Robot tarafı: ham kamera görüntüsünü sıkıştırılmış olarak da yayınlar (image_transport republish).

  <image_topic> (sensor_msgs/Image) ──▶ <image_topic>/compressed (sensor_msgs/CompressedImage)

YOLO düğümü başka bir makinede (sunucu) çalışacaksa ağ üzerinden ham 640×480 rgb8 (~0.9 MB/kare)
yerine JPEG taşınır. Sunucuda:
  ros2 launch fsb_perception perception.launch.py image_transport:=compressed

  ros2 launch fsb_perception image_compress.launch.py use_sim_time:=true [jpeg_quality:=80]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    topic = LaunchConfiguration('image_topic')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('jpeg_quality', default_value='80'),
        Node(package='image_transport', executable='republish', name='image_compress',
             output='screen',
             # Jazzy image_transport: taşıma türleri parametre, topic'ler remap ile verilir
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'in_transport': 'raw', 'out_transport': 'compressed',
                          'out.compressed.jpeg_quality': ParameterValue(
                              LaunchConfiguration('jpeg_quality'), value_type=int)}],
             remappings=[('in', topic), ('out/compressed', [topic, '/compressed'])]),
    ])
