# fsb_bringup

**Amaç:** Tek giriş noktası `bringup.launch.py`. `use_sim` argümanı ile Gazebo veya gerçek robot yolunu seçer; `use_sim_time` buradan tüm düğümlere iletilir.

**Durum:** Simülasyon yolu + SLAM/AMCL/Nav2 çalışıyor (Aşama 5). Gerçek robot yolu Aşama 7.

## Launch
```bash
ros2 launch fsb_bringup bringup.launch.py                  # Gazebo GUI ile
ros2 launch fsb_bringup bringup.launch.py headless:=true   # yalnızca gz sunucusu
ros2 launch fsb_bringup bringup.launch.py headless:=true localization:=slam   # haritalama
ros2 launch fsb_bringup bringup.launch.py headless:=true localization:=amcl   # AMCL + Nav2
```
Sıra: robot_state_publisher → Gazebo + köprü + spawn → `joint_state_broadcaster` spawner
→ (OnProcessExit) `diff_drive_controller` spawner (`~/cmd_vel` → `/cmd_vel` remap) → (OnProcessExit) EKF
→ `localization`'a göre fsb_navigation `slam.launch.py` **veya** `navigation.launch.py` (map→odom tek sahip). Spawner'lar `controller_manager`
servisini bekler (sabit gecikme yok). `rviz:=true` → `rviz/sim.rviz` (LaserScan, kamera, EKF odometrisi; sabit çerçeve `odom`).

## Argümanlar
| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `use_sim` | `true` | `use_sim_time` buna eşitlenir ve tüm düğümlere iletilir |
| `world` | `fsb_gazebo/worlds/factory.sdf` | |
| `headless` | `false` | |
| `rviz` | `false` | RViz aç |
| `rviz_config` | otomatik | `localization:=none` → `rviz/sim.rviz` (sabit çerçeve `odom`; scan, kamera, EKF); `slam`/`amcl` → `rviz/nav.rviz` (sabit çerçeve `map`) |
| `patrol` | `false` | `true` → `fsb_patrol` devriyesini de başlat (yalnızca `localization:=amcl`; değilse yok sayılır ve loglanır) |
| `loops` | `0` | devriye tur sayısı, 0 = sonsuz |
| `localization` | `none` | `none` / `slam` (slam_toolbox haritalama) / `amcl` (map_server + AMCL + Nav2) |
| `map` | `fsb_navigation/maps/factory.yaml` | `localization:=amcl` için harita |

## Devriyeyi gözle izleme (GUI'li Gazebo + RViz + AMCL/Nav2 + devriye)
`rviz/nav.rviz` (sabit çerçeve `map`, üstten görünüm): harita, AMCL parçacıkları (`/particle_cloud`, yeşil oklar),
robot modeli + LaserScan, global plan (`/plan`, mavi), yerel plan (RPP'nin takip ettiği parça
`/received_global_plan`, turuncu), global (soluk) ve lokal costmap, lokal costmap footprint'i (yeşil),
`map/odom/base_footprint` eksenleri, Navigation 2 paneli; araçlar: *2D Pose Estimate*, *Nav2 Goal*.

Her sekmede önce (tüm sekmelerde **aynı** `ROS_DOMAIN_ID`):
```bash
cd ~/factory_security_bot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=85
```

**Önerilen: iki sekme** (önce robotu ve AMCL'yi görür, devriyeyi sonra başlatırsınız)
```bash
# Sekme 1 — Gazebo GUI + RViz (nav.rviz) + EKF + map_server/AMCL + Nav2
ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true

# Sekme 2 — RViz'de parçacıklar robotun etrafında toplandıktan ve Navigation 2 panelinde
#           "active" görünce: 10 turluk devriye (bitince "DEVRİYE BİTTİ: x/10")
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10
```

**Tek komut** (devriye Nav2 hazır olunca kendiliğinden başlar; `loops:=0` → sonsuz):
```bash
ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true patrol:=true loops:=10
```

**İsteğe bağlı sekme 3/4 — ölçüm** (ön planda; Ctrl+C → özet):
```bash
ros2 run fsb_gazebo localization_monitor.py --report 300 --ros-args -p use_sim_time:=true
ros2 run fsb_gazebo recovery_counter.py --ros-args -p use_sim_time:=true
```

Notlar:
- Robot (0,0,0)'da doğar ve AMCL başlangıç pozu da budur (`set_initial_pose: true`); *2D Pose Estimate* gerekmez.
  Yanlış pozdan yakınsamayı izlemek isterseniz RViz'de *2D Pose Estimate* ile bilerek hatalı poz verin.
- Kapatma: launch sekmesinde Ctrl+C (Gazebo, RViz ve Nav2 birlikte kapanır).
- WSLg'de Gazebo GUI + RViz birlikte CPU'yu zorlar; RTF düşerse turlar duvar saatinde uzar (sim süresi
  değişmez, `use_sim_time`). Pencere açılmaz/çökerse: `export LIBGL_ALWAYS_SOFTWARE=1` (yavaş ama çalışır).
  Gazebo GUI gerekmiyorsa `headless:=true` ile yalnızca RViz'den izleyin.

## Teleop
`diff_drive_controller` yalnızca `TwistStamped` kabul eder; teleop_twist_keyboard 2.4.1 `stamped` parametresini destekler (twist_stamper gerekmez):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p speed:=0.2 -p turn:=0.8 -p use_sim_time:=true \
  -r cmd_vel:=/cmd_vel
```
`diff_drive_controller`'ın girişi `/cmd_vel`'dir (Aşama 5'ten beri). `localization:=amcl` iken `/cmd_vel`'i
collision_monitor yayınlar; elle sürüş için Nav2 zincirinin girişini kullanın: `-r cmd_vel:=/cmd_vel_nav`.

## Bilinen durum
- `/gz_ros_control` (gz_ros2_control 1.2.20 eklentisinin yardımcı düğümü) `use_sim_time=false` gösterir; eklenti
  `use_sim_time:=true`'yu yalnızca `controller_manager`'a verir. Kontrol döngüsü (controller_manager + controller'lar) `true`.
- `odom→base_footprint` yalnızca EKF'den gelir (doğrulama: EKF durdurulunca bu TF kesilir, diff_drive aktifken bile).
