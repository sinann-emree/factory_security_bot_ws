# fsb_bringup

**Amaç:** Tek giriş noktası `bringup.launch.py`. `use_sim` argümanı ile Gazebo veya gerçek robot yolunu seçer; `use_sim_time` buradan tüm düğümlere iletilir.

**Durum:** Simülasyon yolu çalışıyor (Aşama 3). Gerçek robot yolu Aşama 7.

## Launch
```bash
ros2 launch fsb_bringup bringup.launch.py                  # Gazebo GUI ile
ros2 launch fsb_bringup bringup.launch.py headless:=true   # yalnızca gz sunucusu
```
Sıra: robot_state_publisher → Gazebo + /clock köprüsü + spawn → `joint_state_broadcaster` spawner
→ (OnProcessExit) `diff_drive_controller` spawner. Spawner'lar `controller_manager` servisini bekler (sabit gecikme yok).

## Argümanlar
| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `use_sim` | `true` | `use_sim_time` buna eşitlenir ve tüm düğümlere iletilir |
| `world` | `fsb_gazebo/worlds/factory.sdf` | |
| `headless` | `false` | |

## Teleop
`diff_drive_controller` yalnızca `TwistStamped` kabul eder; teleop_twist_keyboard 2.4.1 `stamped` parametresini destekler (twist_stamper gerekmez):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p speed:=0.2 -p turn:=0.8 -p use_sim_time:=true \
  -r cmd_vel:=/diff_drive_controller/cmd_vel
```

## Bilinen durum
- `/gz_ros_control` (gz_ros2_control 1.2.20 eklentisinin yardımcı düğümü) `use_sim_time=false` gösterir; eklenti
  `use_sim_time:=true`'yu yalnızca `controller_manager`'a verir. Kontrol döngüsü (controller_manager + controller'lar) `true`.
- Şu an `odom` TF'i yayınlanmıyor (diff_drive `enable_odom_tf: false`); `odom→base_footprint` EKF ile Aşama 4'te gelir.
