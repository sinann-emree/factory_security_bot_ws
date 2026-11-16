"""
Tek giriş noktası: simülasyon veya gerçek robot.

Kullanım:
  ros2 launch fsb_bringup bringup.launch.py                 # use_sim:=true, Gazebo GUI
  ros2 launch fsb_bringup bringup.launch.py headless:=true  # yalnızca gz sunucusu
  ros2 launch fsb_bringup bringup.launch.py use_sim:=false  # gerçek robot (Aşama 7)
  ros2 launch fsb_bringup bringup.launch.py localization:=slam   # haritalama (SLAM Toolbox)
  ros2 launch fsb_bringup bringup.launch.py localization:=amcl   # AMCL + Nav2 (map:=...)
  ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true patrol:=true loops:=5
      # GUI'li Gazebo + RViz (nav.rviz) + AMCL/Nav2 + patrol_executor (idleness), tek komut
      # patrol_mode:=sequential|random|legacy (legacy = eski 4 noktalı patrol_node)
  ros2 launch fsb_bringup bringup.launch.py localization:=amcl patrol:=true perception:=true \
      world:=$(ros2 pkg prefix fsb_gazebo)/share/fsb_gazebo/worlds/factory_lying.sdf
      # + YOLOv8-pose (fsb_perception, ~/fsb_venv) ve yerde yatan insanlı dünya: yatan insan →
      # /security/alerts → patrol_executor olay listesine ekler, çevresinden kayıt alır,
      # devriyeye devam eder, revisit_interval sonra konumu yeniden denetler
      # image_transport:=compressed → YOLO sıkıştırılmış görüntüden çalışır (sunucu düzeni)
      # cmd_vel_watchdog:=true → /cmd_vel → cmd_vel_watchdog → /cmd_vel_safe → diff_drive

Başlatma sırası:
  robot_state_publisher → (sim) Gazebo + bridge + spawn → joint_state_broadcaster spawner
  → (OnProcessExit) diff_drive_controller spawner → (OnProcessExit) EKF (robot_localization)
  → SLAM / Nav2 → (perception:=true) YOLO   [+ isteğe bağlı RViz]
use_sim_time = use_sim (proje tasarım kuralı: use_sim_time tek noktadan, launch argümanıyla
yönetilir).
TF: odom→base_footprint yalnızca EKF (diff_drive enable_odom_tf: false).
    map→odom yalnızca SLAM Toolbox (localization:=slam) VEYA AMCL (localization:=amcl), asla ikisi.
cmd_vel: diff_drive_controller ~/cmd_vel → /cmd_vel (Nav2 collision_monitor çıkışı, teleop);
         cmd_vel_watchdog:=true iken araya 0.5 s'lik komut bekçisi girer (→ /cmd_vel_safe).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = use_sim  # simülasyonda true, gerçek robotta false
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('fsb_control'), 'config', 'controllers.yaml'])
    ekf_yaml = PathJoinSubstitution([FindPackageShare('fsb_control'), 'config', 'ekf.yaml'])

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

    watchdog_on = LaunchConfiguration('cmd_vel_watchdog')
    drive_cmd_topic = PythonExpression(
        ["'/cmd_vel_safe' if '", watchdog_on, "'.lower() == 'true' else '/cmd_vel'"])
    cmd_vel_watchdog = Node(
        package='fsb_control',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'timeout': 0.5}],
        remappings=[('cmd_vel_in', '/cmd_vel'), ('cmd_vel_out', '/cmd_vel_safe')],
        condition=IfCondition(watchdog_on),
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
                   '--param-file', controllers_yaml,
                   # Nav2 collision_monitor çıkışı /cmd_vel → diff_drive (TwistStamped);
                   # bekçi açıksa diff_drive bekçinin çıkışını (/cmd_vel_safe) dinler
                   '--controller-ros-args',
                   ['-r /diff_drive_controller/cmd_vel:=', drive_cmd_topic]],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # EKF: tekerlek odom (vx, vy, vyaw) + IMU (vyaw) → /odometry/filtered + odom→base_footprint TF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
    )

    nav_launch = PathJoinSubstitution([FindPackageShare('fsb_navigation'), 'launch'])
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_launch, 'slam.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'slam')),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_launch, 'navigation.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'map': LaunchConfiguration('map')}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'amcl')),
    )

    # Algı: kendi venv'inde çalışır (bkz. fsb_perception/launch/perception.launch.py)
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_perception'), 'launch', 'perception.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'device': LaunchConfiguration('yolo_device'),
                          'image_transport': LaunchConfiguration('image_transport')}.items(),
        condition=IfCondition(LaunchConfiguration('perception')),
    )
    # Sıkıştırılmış görüntü yolu (sunucu düzeni): ham → <topic>/compressed (robot tarafı)
    image_compress = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_perception'), 'launch', 'image_compress.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('image_transport'),
                                                 'compressed')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Devriye: yalnızca localization:=amcl iken (action sunucuları Nav2'de). Her iki düğüm de
    # action sunucusu hazır olana dek kendisi bekler; sabit gecikme yok.
    #   patrol_mode idleness|sequential|random → patrol_executor (planlayıcı waypoint'leri)
    #   patrol_mode legacy                     → eski 4 noktalı patrol_node (fallback)
    def patrol_cond(legacy: bool) -> IfCondition:
        return IfCondition(PythonExpression([
            "'", LaunchConfiguration('patrol'), "'.lower() == 'true' and '",
            LaunchConfiguration('localization'), "' == 'amcl' and '",
            LaunchConfiguration('patrol_mode'), "' ", '==' if legacy else '!=', " 'legacy'"]))
    patrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_patrol'), 'launch', 'patrol_executor.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'mode': LaunchConfiguration('patrol_mode'),
                          'loops': LaunchConfiguration('loops'),
                          'coverage': LaunchConfiguration('coverage'),
                          'map': LaunchConfiguration('map'),
                          'revisit_interval': LaunchConfiguration('revisit_interval'),
                          'alert_log_dir': LaunchConfiguration('security_log_dir')}.items(),
        condition=patrol_cond(legacy=False),
    )
    patrol_legacy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fsb_patrol'), 'launch', 'patrol.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'loops': LaunchConfiguration('loops')}.items(),
        condition=patrol_cond(legacy=True),
    )
    patrol_ignored = LogInfo(
        msg='patrol:=true yok sayıldı: devriye için localization:=amcl gerekli.',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('patrol'), "'.lower() == 'true' and '",
            LaunchConfiguration('localization'), "' != 'amcl'"])),
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
        DeclareLaunchArgument(
            'localization', default_value='none', choices=['none', 'slam', 'amcl'],
            description='map→odom kaynağı: none | slam (haritalama) | amcl (+ Nav2)'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fsb_navigation'), 'maps', 'factory.yaml']),
            description='localization:=amcl için harita yaml'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true: RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('fsb_bringup'), 'rviz',
                PythonExpression(["'sim.rviz' if '", LaunchConfiguration('localization'),
                                  "' == 'none' else 'nav.rviz'"])]),
            description='RViz yapılandırması. Varsayılan: localization:=none → sim.rviz '
                        '(odom; scan, kamera, EKF), aksi halde nav.rviz (map; harita, AMCL '
                        "parçacıkları, planlar, costmap'ler)"),
        DeclareLaunchArgument(
            'patrol', default_value='false',
            description='true: fsb_patrol devriyesini de başlat (localization:=amcl gerekli)'),
        DeclareLaunchArgument(
            'patrol_mode', default_value='idleness',
            choices=['idleness', 'sequential', 'random', 'legacy'],
            description='patrol_executor modu; legacy = eski 4 noktalı patrol_node'),
        DeclareLaunchArgument(
            'loops', default_value='0', description='devriye tur sayısı, 0 = sonsuz'),
        DeclareLaunchArgument(
            'coverage', default_value='false',
            description='true: coverage_monitor da başlar (patrol_executor ile)'),
        DeclareLaunchArgument(
            'perception', default_value='false',
            description='true: YOLOv8 algı düğümü (fsb_perception; ~/fsb_venv gerekli)'),
        DeclareLaunchArgument(
            'image_transport', default_value='raw', choices=['raw', 'compressed'],
            description='compressed: kamera görüntüsü JPEG olarak da yayınlanır ve YOLO onu '
                        'dinler (YOLO başka makinede çalışacaksa)'),
        DeclareLaunchArgument(
            'cmd_vel_watchdog', default_value='false',
            description='true: /cmd_vel → cmd_vel_watchdog (0.5 s komut gelmezse dur) → '
                        'diff_drive'),
        DeclareLaunchArgument(
            'revisit_interval', default_value='300.0',
            description='yatan insan kaydından sonra revisit süresi (sim-s)'),
        DeclareLaunchArgument(
            'security_log_dir', default_value='~/.ros/fsb_security',
            description='olay listesi (watchlist.yaml), events.jsonl ve kareler'),
        DeclareLaunchArgument(
            'yolo_device', default_value='',
            description="YOLO cihazı: '' = CUDA varsa cuda:0, yoksa cpu | cpu | cuda:0"),

        rsp,
        sim,
        real_placeholder,
        jsb_spawner,
        # Sıra: broadcaster aktif olduktan sonra diff_drive
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[ddc_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=ddc_spawner, on_exit=[ekf])),
        slam,
        nav2,
        cmd_vel_watchdog,
        image_compress,
        perception,
        rviz,
        patrol,
        patrol_legacy,
        patrol_ignored,
    ])
