# Fabrika Otonom Güvenlik ve Devriye Robotu

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![License](https://img.shields.io/badge/license-Apache--2.0-blue)

> **English summary.** An autonomous security and patrol robot for factory floors, built on
> ROS 2 Jazzy and Gazebo Harmonic. A differential-drive robot (Andino-class dimensions) with a 360°
> LiDAR, RGB camera and IMU uses ros2_control (identical stack for simulation and real hardware),
> EKF odometry fusion, SLAM Toolbox / AMCL / Nav2, and a patrol planner that generates waypoints
> automatically from the map (medial-axis skeleton + greedy set cover + TSP) and executes them with an
> idleness-based strategy. Simulation stages 0–5b are complete and verified; YOLOv8 perception
> (stage 6) and the real-robot port (stage 7) are planned.

**ROS 2 Jazzy + Gazebo Harmonic tabanlı, fabrika zemininde otonom devriye gezen güvenlik robotu.**

![Haritadan otomatik üretilen devriye noktaları](docs/images/waypoints_factory.png)

---

## İçindekiler
- [Öne çıkan özellikler](#öne-çıkan-özellikler)
- [Mimari](#mimari)
- [Kurulum](#kurulum)
- [Çalıştırma](#çalıştırma)
- [Durum](#durum)
- [Atıflar](#atıflar)
- [Lisans](#lisans)

---

## Öne çıkan özellikler

Aşağıdaki tüm sayılar simülasyonda (Gazebo Harmonic, 20×14 m fabrika dünyası) ölçülmüştür;
kaynakları geliştirme sırasında alınan ölçümler ve commit geçmişidir.

### Sim/real uyumlu ros2_control mimarisi
- Tek URDF/xacro; `use_sim` argümanı yalnızca donanım eklentisini değiştirir
  (`gz_ros2_control/GazeboSimSystem` ↔ gerçek robot hardware interface). Nav2, SLAM ve devriye katmanı
  iki ortamda da aynıdır.
- Fiziksel parametreler tek kaynakta (`fsb_properties.xacro`); tekerlek yarıçapı 0.0331 m,
  tekerlek ayrımı 0.128 m. `test_params_consistency.py` URDF ↔ `controllers.yaml` uyumunu test eder.
- `joint_state_broadcaster` + `diff_drive_controller` (`cmd_vel` = `TwistStamped`, doğrulandı).
- 1 m ileri komutunda odom 1.0260 m = gerçek 1.0260 m; ±360° dönüşte odom–gerçek farkı
  −0.003° / +0.002°.
- LiDAR 180° dönük monte (`rpy="0 0 π"`); scan hizalama testinde doğru montaj %97.9 / %98.5 inlier,
  yanlış (+180°) varsayım %1.5 / %5.7.

### EKF ile odometri birleştirme
- `robot_localization` EKF: tekerlek odometrisi (vx, vy, vyaw) + IMU (vyaw), 30 Hz.
- `odom → base_footprint` TF'inin **tek** yayıncısı EKF'dir (`enable_odom_tf: false`).
- 5 dk rastgele sürüş drift testi (~44 m, ~3000° toplam dönüş): kapalı çevrim hatası
  0.18 cm / +0.11° ve 1.08 cm / −0.26°.

### SLAM Toolbox + AMCL + Nav2
- SLAM Toolbox ile haritalama; harita kalitesi (dünyaya karşı): dolu hücre → yüzey medyan 2.2 cm,
  p95 5.5 cm, %99.96'sı ≤ 10 cm, çift duvar yok, kapsama %99.1.
- AMCL: +60 cm / 16.5° hatalı ilk pozdan 7.0 sim-saniyede 1.2 cm / 0.5° hataya yakınsama.
- Nav2: SmacPlanner2D global planlayıcı, Regulated Pure Pursuit, velocity_smoother, collision_monitor.
  NavFn 810 waypoint çiftinin 21'inde yol bulamazken SmacPlanner2D 0/810.
- 4 noktalı devriyede 10/10 tur başarılı, 0 kurtarma davranışı; dinamik engel testinde (rotaya
  konan 0.5 m kutu) hedefe ulaşıldı, minimum açıklık 38.8 cm.

![SLAM Toolbox ile çıkarılan fabrika haritası](docs/images/map_factory.png)

### Haritadan otomatik waypoint üretimi (`patrol_planner`)
Kodda hiçbir dünya koordinatı yoktur; girdi yalnızca harita + `robot_params.yaml`.
1. Serbest alan robot yarıçapı + güvenlik payı kadar şişirilir.
2. Medial axis iskeleti (Zhang–Suen inceltme; genelleştirilmiş Voronoi diyagramının ızgara
   yaklaşıklığı) çıkarılır ve kısa dallar budanır; kavşak, uç ve örnek noktalar aday olur.
3. Her koridor segmentine (raf araları) en az bir waypoint zorunlu tutulur.
4. Kamera FOV'u + ışın izleme ile tembel greedy **set cover** (hedef ≥ %95).
5. Costmap/açıklık doğrulaması, ardından Nav2 gerçek yol uzunluklarıyla **TSP**
   (n ≤ 12 Held–Karp, aksi halde çok başlangıçlı NN + 2-opt + Or-opt).

Fabrika haritasında sonuç: **29 waypoint**, planlanan kapsama **%95.02** (dar bölge %93.7, geniş
%98.3), 12/12 koridor segmentine giriş, min açıklık 0.475 m, TSP turu 87.4 m. Döndürülmüş /
aynalanmış / kırpılmış 7 sentetik harita varyantında (Gazebo'suz, `colcon test` içinde) 12–31
waypoint ve %95.02–95.65 kapsama.

Yalnızca apt paketleri kullanılır (`python3-numpy`, `python3-scipy`, `python3-yaml`,
`python3-matplotlib`); scikit-image, networkx ve pip OpenCV bilinçli olarak kullanılmadı.

### Idleness tabanlı devriye (`patrol_executor`)
- Varsayılan mod `idleness`: skor = ağırlık · bekleme süresi − tahmini yol süresi. Ayrıca
  `sequential` (TSP sırası) ve `random` modları.
- Ulaşılamayan hedef atlanır ve loglanır; öncelikli noktalar ayrı yaml'da ağırlıklıdır.
- RViz'de `/patrol/waypoints` MarkerArray ile durum renkleri (hedef / ziyaret / atlandı / bekliyor).
- 5 turluk idleness devriyesi: **5/5 tur eksiksiz**, 156 ziyaret, 0 atlama, 0 kurtarma,
  **0 çarpışma** (ground truth, 528 559 örnek), min açıklık 10.5 cm, ~734 m yol.
  `map→odom` poz sıçraması maks 4.71 cm (p99 1.87 cm).

### Kapsama izleme (`coverage_monitor`)
Robotun gerçek yolundan, simüle kamera görüşüyle kapsamayı ölçer. 5 turluk devriyede gerçek yoldan
kapsama **%99.95** (dar bölge %99.94), 12/12 koridor.

![Gerçek yoldan ölçülen kapsama](docs/images/coverage_real.png)

---

## Mimari

### Paketler
| Paket | Tür | İçerik |
|---|---|---|
| `fsb_description` | ament_cmake | URDF/xacro, sensörler, `<ros2_control>` (sim/real), RViz |
| `fsb_control` | ament_cmake | `controllers.yaml`, `ekf.yaml` (gerçek robot hw interface: planlanıyor) |
| `fsb_gazebo` | ament_cmake | `factory.sdf` dünyası, `gz_bridge.yaml`, sim launch, test/ölçüm betikleri |
| `fsb_navigation` | ament_cmake | SLAM Toolbox, AMCL, Nav2 parametreleri ve launch, kayıtlı harita |
| `fsb_bringup` | ament_cmake | Tek giriş noktası `bringup.launch.py`, RViz konfigürasyonları |
| `fsb_patrol` | ament_python | `patrol_planner`, `patrol_executor`, `coverage_monitor` |
| `fsb_perception` | ament_python | YOLOv8 düğümü (iskelet; Aşama 6'da planlanıyor) |

Her paketin kendi `README.md` dosyasında launch komutları ve parametreler bulunur.

```mermaid
flowchart LR
    bringup[fsb_bringup] --> desc[fsb_description]
    bringup --> gz[fsb_gazebo]
    bringup --> ctrl[fsb_control]
    bringup --> nav[fsb_navigation]
    bringup --> patrol[fsb_patrol]
    gz -- "/scan /imu /camera /clock" --> ctrl
    ctrl -- "EKF: odom→base_footprint" --> nav
    nav -- "NavigateToPose" --> patrol
    perc["fsb_perception<br/>(planlanıyor)"] -.-> patrol
```

### TF ağacı
Her dönüşümün tek bir sahibi vardır: `map→odom` SLAM Toolbox **veya** AMCL, `odom→base_footprint`
yalnızca EKF.

```mermaid
flowchart TD
    map -->|"SLAM Toolbox / AMCL"| odom
    odom -->|EKF| base_footprint
    base_footprint --> base_link
    base_link --> lidar_link["lidar_link (yaw 180°)"]
    base_link --> camera_link --> camera_optical_link
    base_link --> imu_link
    base_link --> left_wheel_link
    base_link --> right_wheel_link
    base_link --> caster_link
```

---

## Kurulum

Hedef ortam: Ubuntu 24.04 (WSL2 üzerinde test edildi), ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8).

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard liburdfdom-tools \
  python3-numpy python3-scipy python3-yaml python3-matplotlib
```

Depoyu klonlayıp derleyin (`--symlink-install` **kullanılmaz**):

```bash
git clone https://github.com/sinann-emree/factory_security_bot_ws.git ~/factory_security_bot_ws
cd ~/factory_security_bot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y   # isteğe bağlı: eksik bağımlılıkları tamamlar
colcon build
source install/setup.bash
```

Testler:
```bash
colcon test && colcon test-result --verbose
```

---

## Çalıştırma

Tüm sistem tek launch dosyasından başlar; `use_sim` (varsayılan `true`) `use_sim_time`'ı tüm
düğümlere iletir.

```bash
# Gazebo + controller'lar + EKF
ros2 launch fsb_bringup bringup.launch.py

# Haritalama (SLAM Toolbox)
ros2 launch fsb_bringup bringup.launch.py localization:=slam rviz:=true

# AMCL + Nav2 + 5 turluk idleness devriyesi (tek komut)
ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true patrol:=true loops:=5
```

Sık kullanılan argümanlar: `headless:=true` (yalnızca gz sunucusu), `patrol_mode:=idleness|sequential|random|legacy`,
`coverage:=true`, `map:=<harita.yaml>`. Ayrıntılar: [`src/fsb_bringup/README.md`](src/fsb_bringup/README.md).

Harita değişince waypoint'leri yeniden üretmek için: [`src/fsb_patrol/README.md`](src/fsb_patrol/README.md).

> **WSL2 notu:** Gazebo GUI + RViz pencereleri açılmazsa `export LIBGL_ALWAYS_SOFTWARE=1` deneyin
> veya `headless:=true` ile yalnızca RViz'den izleyin.

---

## Durum

| Aşama | Kapsam | Durum |
|---|---|---|
| 0 | Ortam hazırlığı (ROS 2 Jazzy, Gazebo Harmonic) | ✅ Tamamlandı |
| 1 | Paket iskeletleri (7 `fsb_*` paketi) | ✅ Tamamlandı |
| 2 | URDF/xacro ve ros2_control | ✅ Tamamlandı |
| 3 | Gazebo entegrasyonu, köprü, controller'lar | ✅ Tamamlandı |
| 4 | Sensörler (LiDAR, kamera, IMU) ve EKF | ✅ Tamamlandı |
| 5 | SLAM Toolbox, AMCL, Nav2, waypoint devriyesi | ✅ Tamamlandı |
| 5b | Haritadan otomatik kapsamlı devriye | ✅ Tamamlandı |
| 6 | YOLOv8 ile insan/anomali/ihlal tespiti, `/security/alerts` | 🗓️ **Planlanıyor** |
| 7 | Gerçek robota geçiş (`fsb_hardware`, `use_sim:=false`, kalibrasyon) | 🗓️ **Planlanıyor** |


---

## Atıflar

- [Andino (Ekumen-OS/andino)](https://github.com/Ekumen-OS/andino) — yalnızca robot **ölçüleri ve
  fiziksel parametreler** için referans olarak kullanılmıştır (commit `974264b`). Andino deposundan
  hiçbir dosya kopyalanmamıştır; tüm `fsb_*` paketleri sıfırdan yazılmıştır ve kullanılan her değerin
  kaynağı ilgili dosyada yorum satırıyla belirtilmiştir.
- ROS 2, Gazebo, ros2_control, Nav2, SLAM Toolbox ve robot_localization topluluklarına teşekkürler.

## Yazar

**Sinan Emre Gülşen** — GitHub: [@sinann-emree](https://github.com/sinann-emree)

## Lisans

Bu proje [Apache License 2.0](LICENSE) ile lisanslanmıştır.
