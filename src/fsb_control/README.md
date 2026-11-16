# fsb_control

**Amaç:** `controllers.yaml` (joint_state_broadcaster + diff_drive_controller), EKF (`robot_localization`) yaml'ı; Aşama 7'de gerçek robot donanım arayüzü.

**Durum:** `controllers.yaml` tamam (Aşama 2). EKF Aşama 4.

## Dizinler
- `config/` — `controllers.yaml` (Aşama 2), `ekf.yaml` (Aşama 4)
- `launch/`

## Parametre notları (kurulu sürümden doğrulandı: ros-jazzy-diff-drive-controller 4.42.1)
- `cmd_vel` tipi: **yalnızca `geometry_msgs/TwistStamped`** (`diff_drive_controller.hpp:126`).
- `use_stamped_vel` parametresi bu sürümde **yok** → yaml'a yazılmaz.
- `wheel_radius`/`wheel_separation`: `fsb_description` xacro property'si ile birebir (0.0331 / 0.128).
- `enable_odom_tf: false` (TF'i EKF yayınlar), `base_frame_id: base_footprint`.
- `has_*_limits` deprecated → yazılmaz; yazılmayan sınır NaN = sınırsız. Kurallar: `min_velocity ≤ 0`, `max_acceleration ≥ 0`, `max_deceleration ≤ 0`.
- Sınırlar: lineer ±0.3 m/s, ±0.5 m/s²; açısal ±1.0 rad/s, ±1.5 rad/s² (proje seçimi, Aşama 5'te Nav2 ile ayarlanır).
- `update_rate` / `publish_rate`: 50 Hz (andino).
