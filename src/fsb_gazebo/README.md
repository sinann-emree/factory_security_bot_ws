# fsb_gazebo

**Amaç:** Gazebo Harmonic (gz-sim 8) fabrika dünyası, `ros_gz_bridge` yaml'ı ve simülasyon launch dosyası.

**Durum:** Aşama 5 tamam (LiDAR, kamera, IMU köprüleri; SLAM/AMCL/Nav2 test araçları).

## Dizinler
- `worlds/factory.sdf` — 20×14 m salon, 3 m duvarlar; raf sıraları y=±3.0 / ±5.5 (4×0.8×2 m);
  ara koridorlar x∈[-4,-2], [2,4]; 4 kolon; doğu ucunda paletler. Ana koridor |y|<2.6 m boş, robot (0,0)'da başlar.
  Sistemler: Physics (1 ms adım), UserCommands (spawn için gerekli), SceneBroadcaster,
  Sensors (`ogre2`; ekran yoksa otomatik headless/EGL), Imu.
- `models/` — (boş) harici modeller için
- `config/gz_bridge.yaml` — `/clock`, `/scan`, `/camera/image_raw`, `/camera/camera_info`, `/imu` (hepsi GZ→ROS)
- `scripts/` — test araçları (aşağıda)
- `launch/sim.launch.py` — gz sim + bridge + `ros_gz_sim create` (`/robot_description` topic'inden)

## Launch
Genelde `fsb_bringup` üzerinden. Doğrudan (robot_state_publisher ayrıca çalışıyor olmalı):
```bash
ros2 launch fsb_gazebo sim.launch.py headless:=true
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `world` | `worlds/factory.sdf` | SDF dünya |
| `headless` | `false` | `true` → `gz sim -s` (GUI yok) |
| `x`,`y`,`z`,`yaw` | `0`,`0`,`0.01`,`0` | Spawn pozu |

## Sensör topic'leri
| ROS topic | Tip | frame_id | Hız |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `lidar_link` (180° dönük) | 10 Hz, 360 ışın, 0.15–12 m |
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8 640×480) | `camera_optical_link` | 15 Hz hedef (headless ~13 Hz) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_optical_link` | 15 Hz |
| `/imu` | `sensor_msgs/Imu` | `imu_link` | 100 Hz |

## Test araçları (`ros2 run fsb_gazebo <script>`, simülasyon çalışırken)
Gerçek konum (ground truth) doğrudan gz-transport ile `/world/factory/dynamic_pose/info`'dan okunur;
köprüye ground truth eklenmez.

| Araç | Ne yapar |
|---|---|
| `drive_test.py basic` | 1 m ileri, ±360° dönüş: tekerlek odom ↔ gerçek konum (Aşama 3 kriteri) |
| `drive_test.py drift --duration 300 --seed 1` | Serbest alanda rastgele hedefler + yerinde dönüşler, sonra EKF'ye göre başlangıca dönüş. EKF ve tekerlek odom'un drift'i (son, duraklarken maks., hareket halinde maks.) raporlanır (Aşama 4 kriteri: < 10 cm, < 5°). Hedef kutusu `--box-x/--box-y` (varsayılan ±3.5 / ±1.5 m, başlangıca göre; robot (0,0)'dan başlamalı) |
| `teleop_test.py` | teleop_twist_keyboard'u pty'de çalıştırıp tuş gönderir (stamped zinciri uçtan uca) |
| `mapping_tour.py` | SLAM haritalaması için önceden tanımlı yavaş tur (0.15 m/s, ≤ 0.4 rad/s), EKF'ye göre noktadan noktaya; `/cmd_vel`'e yazar (Nav2 kapalıyken, `localization:=slam`) |
| `map_quality_check.py --map <yaml>` | Kaydedilmiş haritayı SDF kutularıyla karşılaştırır: dolu hücre → gerçek yüzey uzaklığı (medyan/p95, >10 cm = çift duvar), global hizalama (dθ, dx, dy), duvar kalınlığı, kapsama |
| `amcl_check.py` | Kasıtlı hatalı `/initialpose` (+0.5, -0.4 m, +20°) verir, robotu `/cmd_vel_nav` zinciriyle döndürüp sürer; AMCL hatası, kovaryans ve parçacık yayılımı zamana göre; yakınsama süresi |
| `localization_monitor.py` | map→odom adımları (sıçrama sayısı, maks Δxy/Δyaw) ve map→base_footprint ↔ gerçek konum hatası; Ctrl+C / `--duration` ile özet |
| `dynamic_obstacle_test.py` | NavigateToPose A→B sırasında rotaya haritada olmayan 0.5 m kutu spawn eder (`/world/factory/create`); min. açıklık, yanal sapma, kutu yer değiştirmesi (çarpışma) ölçer, kutuyu siler |
| `scan_alignment_check.py` | Her `/scan` noktasını gerçek poz × TF ile dünyaya yansıtıp SDF kutularına uzaklığı ölçer; aynısını +180° yanlış montaj varsayımıyla da yapar. Simetrik olmayan bir pozda çalıştırın (ör. `gz service -s /world/factory/set_pose ...`) |
| `recovery_counter.py` | Nav2 kurtarma davranışlarını (spin/backup/drive_on_heading/wait/assisted_teleop) `_action/status` topic'lerinden benzersiz goal_id ile sayar; her çağrıda `KURTARMA:` uyarısı, Ctrl+C'de `KURTARMA ÖZETİ`. ClearCostmap (servis) sayılmaz. Devriye testinde `localization_monitor.py` ile birlikte çalıştırılır |
| `gt_pose_publisher.py` | Gerçek konumu `/gt_pose` (`nav_msgs/Odometry`, sim zamanı) olarak yayınlar — **yalnızca rosbag kaydı için**, navigasyon kullanmaz |
| `slam_replay_eval.py` | Bag oynatılırken SLAM `map→base_footprint` ↔ `/gt_pose` hatası (ort/p95/maks). SLAM parametrelerini Gazebo'suz, aynı veriyle karşılaştırmak için (akış betiğin docstring'inde) |
| `clearance_monitor.py [--duration 0]` | Gerçek poz + footprint (nav2_params) ile SDF kutularına en kısa mesafe (1 cm kenar örnekleri): min açıklık, çarpışma (≤ 0) örnek sayısı, < 2 cm süre, açıklık dağılımı; ayrıca engellerin itilip itilmediği. SIGINT/SIGTERM'de (arka planda da) `SONUÇ: ÇARPIŞMA YOK/VAR` |
| `generate_factory_world.py <çıktı.sdf>` | `worlds/factory.sdf` üreticisi (çıktı depodaki dosyayla bayt bayt aynı). Engel yerleşimi buradan değiştirilir; kalite/hizalama araçları geometriyi SDF'ten okur |

### Örnek: devriye testi (Aşama 5 çıkış kriteri)
```bash
ros2 launch fsb_bringup bringup.launch.py headless:=true localization:=amcl
# her biri ayrı terminalde (ön planda) — bitince Ctrl+C → SON ÖZET / KURTARMA ÖZETİ
ros2 run fsb_gazebo localization_monitor.py --report 300 --ros-args -p use_sim_time:=true
ros2 run fsb_gazebo recovery_counter.py --ros-args -p use_sim_time:=true
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10
```
Dikkat: `&` ile arka plana atılan süreçler betik içinde SIGINT'i yok sayar; `localization_monitor.py`
o durumda `kill -INT` ile özet basmaz. Arka planda çalıştıracaksanız `--duration <sim-s>` verin
(10 tur ≈ 1400 sim-s → ör. `--duration 1500`). `recovery_counter.py` kendi sinyal işleyicisini kurar, etkilenmez.

## Kurallar
- Yalnızca `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`; Gazebo Classic yok.
- `/clock` yalnızca köprüden gelir.
