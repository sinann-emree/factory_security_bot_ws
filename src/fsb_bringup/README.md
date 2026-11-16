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
| `rviz` | `false` | RViz (scan + kamera + EKF) |
| `localization` | `none` | `none` / `slam` (slam_toolbox haritalama) / `amcl` (map_server + AMCL + Nav2) |
| `map` | `fsb_navigation/maps/factory.yaml` | `localization:=amcl` için harita |

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
