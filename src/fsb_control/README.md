# fsb_control

**Amaç:** `controllers.yaml` (joint_state_broadcaster + diff_drive_controller), EKF (`robot_localization`) yaml'ı; Aşama 7'de gerçek robot donanım arayüzü.

**Durum:** `controllers.yaml` (Aşama 2) ve `ekf.yaml` (Aşama 4) tamam.

## EKF (`config/ekf.yaml`, robot_localization 3.8.3)
- `odom→base_footprint` TF'inin **tek** yayıncısı (`publish_tf: true`; diff_drive `enable_odom_tf: false`).
- Girdiler: tekerlek odom `vx, vy, vyaw` + IMU **yalnızca `vyaw`**. Mutlak IMU yaw kullanılmaz
  (Gazebo'da fiilen ground truth olurdu; gerçekte manyetometre yoksa güvenilmez; sim = real konfigürasyon).
- Çıktı: `/odometry/filtered` (30 Hz). `two_d_mode: true`.

## `cmd_vel_watchdog` (`scripts/cmd_vel_watchdog.py`)
Robot tarafında çalışan hız komutu rölesi: `cmd_vel_in` → `cmd_vel_out` (`TwistStamped`). Son
komuttan beri `timeout` (0.5 s) geçerse sıfır hız yayınlar ve komut yeniden gelene dek `zero_rate`
(10 Hz) ile yineler. Navigasyon/algı başka bir makinede (sunucu) çalışırken bağlantı koparsa
robotun son komutla yürümesini önler. Zaman aşımı komutun **alındığı** ana göre (düğümün saati)
ölçülür; sunucu–robot saat farkından etkilenmez. Durum: `~/timed_out` (`std_msgs/Bool`).
```bash
ros2 run fsb_control cmd_vel_watchdog.py --ros-args -r cmd_vel_in:=/cmd_vel \
    -r cmd_vel_out:=/cmd_vel_safe -p use_sim_time:=true
# bringup ile: ... cmd_vel_watchdog:=true  (diff_drive girişi /cmd_vel_safe olur)
```

## Dizinler
- `config/` — `controllers.yaml` (Aşama 2), `ekf.yaml` (Aşama 4)
- `scripts/` — `cmd_vel_watchdog.py`
- `launch/`

## Parametre notları (kurulu sürümden doğrulandı: ros-jazzy-diff-drive-controller 4.42.1)
- `cmd_vel` tipi: **yalnızca `geometry_msgs/TwistStamped`** (`diff_drive_controller.hpp:126`).
- `use_stamped_vel` parametresi bu sürümde **yok** → yaml'a yazılmaz.
- `wheel_radius`/`wheel_separation`: `fsb_description` xacro property'si ile birebir (0.0331 / 0.128).
- `enable_odom_tf: false` (TF'i EKF yayınlar), `base_frame_id: base_footprint`.
- `has_*_limits` deprecated → yazılmaz; yazılmayan sınır NaN = sınırsız. Kurallar: `min_velocity ≤ 0`, `max_acceleration ≥ 0`, `max_deceleration ≤ 0`.
- Sınırlar: lineer ±0.2 m/s, ±0.5 m/s²; açısal ±1.0 rad/s, ±1.5 rad/s² (proje seçimi; 0.2 m/s hedef motorların RPM bütçesinden, bkz. `docs/HARDWARE.md`).
- `update_rate` / `publish_rate`: 50 Hz (andino).
