# fsb_control

**Amaç:** `controllers.yaml` (joint_state_broadcaster + diff_drive_controller), EKF (`robot_localization`) yaml'ı; Aşama 7'de gerçek robot donanım arayüzü.

**Durum:** İskelet (Aşama 1).

## Dizinler
- `config/` — `controllers.yaml` (Aşama 2), `ekf.yaml` (Aşama 4)
- `launch/`

## Parametre notları (kurulu sürümden doğrulandı: ros-jazzy-diff-drive-controller 4.42.1)
- `cmd_vel` tipi: **yalnızca `geometry_msgs/TwistStamped`** (`diff_drive_controller.hpp:126`).
- `use_stamped_vel` parametresi bu sürümde **yok** → yaml'a yazılmaz.
- `wheel_radius`/`wheel_separation`: `fsb_description` xacro property'si ile birebir (0.0331 / 0.128).
- `enable_odom_tf: false` (TF'i EKF yayınlar), `base_frame_id: base_footprint`.
