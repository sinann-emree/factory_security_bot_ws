"""
YOLOv8 algı düğümü (fsb_yolo_node).

  ros2 launch fsb_perception perception.launch.py use_sim_time:=true [device:=cpu|cuda:0]
      [model_path:=~/yolov8n-pose.pt] [venv:=~/fsb_venv] [params_file:=...]
      [image_transport:=raw|compressed] [image_topic:=/camera/image_raw]

Başka makinede (sunucu) çalıştırma: bu launch dosyası yalnızca YOLO düğümünü başlatır; TF, tarama
ya da robot sürücüsü gerektirmez. Sunucuda `image_transport:=compressed` verin; robot tarafında
`image_compress.launch.py` ham görüntüyü <image_topic>/compressed olarak yayınlar.

Python ortamı: düğüm `venv` içindeki yorumlayıcıyla ve PYTHONNOUSERSITE=1 ile çalıştırılır.
  - ultralytics/torch yalnızca venv'de kuruludur (opencv-python OLMADAN).
  - PYTHONNOUSERSITE=1 ~/.local'daki pip OpenCV 5'i yoldan çıkarır; cv2 sistemdeki apt OpenCV 4.6
    olur (cv_bridge bununla derlendi). ~/.local'a dokunulmaz.
  - YOLO_AUTOINSTALL=false: ultralytics eksik paket görürse kendiliğinden pip çalıştırmasın
    (opencv-python'u venv'e kurup aynı gölgelemeyi geri getirirdi).
  - OMP_NUM_THREADS / OMP_WAIT_POLICY=PASSIVE: torch'un OpenMP havuzu tüm çekirdeklere yayılıp
    boşta dönerek (spin) CPU yakmasın. Sınırsızken düğüm 5 Hz'te ~2.8 çekirdek kullanıyor ve
    açılıştaki yük Nav2 lifecycle geçişlerini zaman aşımına düşürebiliyordu.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    venv_python = PathJoinSubstitution([LaunchConfiguration('venv'), 'bin', 'python3'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('venv', default_value=os.path.expanduser('~/fsb_venv'),
                              description='ultralytics kurulu venv (--system-site-packages)'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('fsb_perception'), 'config', 'yolo.yaml'])),
        DeclareLaunchArgument('omp_threads', default_value='2',
                              description='YOLO sürecinin OpenMP iş parçacığı üst sınırı'),
        DeclareLaunchArgument('model_path', default_value='~/yolov8n-pose.pt'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('image_transport', default_value='raw',
                              choices=['raw', 'compressed'],
                              description='compressed: <image_topic>/compressed dinlenir'),
        DeclareLaunchArgument('device', default_value='',
                              description="'' = CUDA varsa cuda:0, yoksa cpu"),
        Node(package='fsb_perception', executable='yolo_node', name='fsb_yolo_node',
             output='screen',
             prefix=[venv_python],
             additional_env={'PYTHONNOUSERSITE': '1', 'YOLO_AUTOINSTALL': 'false',
                             'OMP_NUM_THREADS': LaunchConfiguration('omp_threads'),
                             'OMP_WAIT_POLICY': 'PASSIVE'},
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'model_path': ParameterValue(LaunchConfiguration('model_path'),
                                                       value_type=str),
                          'device': ParameterValue(LaunchConfiguration('device'),
                                                   value_type=str),
                          'image_topic': LaunchConfiguration('image_topic'),
                          'image_transport': LaunchConfiguration('image_transport')}]),
    ])
