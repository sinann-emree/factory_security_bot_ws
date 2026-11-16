# fsb_bringup

**Amaç:** Tek giriş noktası `bringup.launch.py`. `use_sim` argümanı ile Gazebo veya gerçek robot yolunu seçer; `use_sim_time` buradan tüm düğümlere iletilir.

**Durum:** Simülasyon yolu + SLAM/AMCL/Nav2 + devriye + YOLOv8 algısı çalışıyor (Aşama 6). Gerçek robot yolu Aşama 7.

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
| `perception` | `false` | `true` → `fsb_perception` YOLOv8 düğümü (`~/fsb_venv` gerekli, bkz. `fsb_perception/README.md`) |
| `yolo_device` | otomatik | `cpu` / `cuda:0`; verilmezse CUDA varsa `cuda:0` |
| `localization` | `none` | `none` / `slam` (slam_toolbox haritalama) / `amcl` (map_server + AMCL + Nav2) |
| `map` | `fsb_navigation/maps/factory.yaml` | `localization:=amcl` için harita |

## Devriyeyi gözle izleme (GUI'li Gazebo + RViz + AMCL/Nav2 + patrol_executor)
`rviz/nav.rviz` (sabit çerçeve `map`, üstten görünüm):
- harita, global (soluk) ve lokal costmap
- AMCL parçacıkları (`/particle_cloud`, yeşil oklar)
- robot modeli, LaserScan, lokal costmap footprint'i (yeşil)
- global plan (`/plan`, mavi), yerel plan (RPP'nin izlediği parça `/received_global_plan`, turuncu)
- **devriye waypoint'leri** (`/patrol/waypoints`, MarkerArray: yön oku + `W01..`, `P_..` etiketi).
  Renkler: **turuncu, büyük** = şu anki hedef, **yeşil** = bu turda ziyaret edildi,
  **kırmızı** = bu turda atlandı, **mavi** = bekliyor, **mor** = bekleyen öncelikli nokta.
  Her tur başında renkler maviye/mora döner. **Kırmızı küre + `ALARM #n`** = güvenlik alarmının verildiği konum.
- **YOLO görüntüsü** (`/perception/image_annotated`; yalnızca `perception:=true` iken dolar).
- `map/odom/base_footprint` eksenleri, Navigation 2 paneli; araçlar: *2D Pose Estimate*, *Nav2 Goal*.

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

# Sekme 2 — Navigation 2 panelinde "active" görünce: 5 turluk idleness devriyesi
#           (her tur "Tur k TAMAM", sonunda "DEVRİYE BİTTİ")
ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true loops:=5
```
Waypoint'ler RViz'de executor başlar başlamaz görünür (Nav2'yi beklerken bile). Diğer modlar:
`mode:=sequential` (TSP sırası) veya `mode:=random`; `loops:=0` → sonsuz;
`coverage:=true coverage_png:=$HOME/coverage.png` → gerçek yoldan kapsama da ölçülür (Ctrl+C'de özet + PNG).

**Tek komut** (devriye Nav2 hazır olunca kendiliğinden başlar):
```bash
ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true patrol:=true loops:=5
#   patrol_mode:=idleness|sequential|random (varsayılan idleness)  coverage:=true
#   patrol_mode:=legacy → eski 4 noktalı patrol_node (fallback)
```

**YOLOv8 ile insan tespiti ve alarm** (insan modelli dünya; ilk açılışta model Fuel'dan iner):
```bash
ros2 launch fsb_bringup bringup.launch.py localization:=amcl rviz:=true patrol:=true perception:=true \
    world:=$(ros2 pkg prefix fsb_gazebo)/share/fsb_gazebo/worlds/factory_person.sdf
```
Robot insanı (doğu yükleme alanı, paletlerin arası) ≥ %80 güvenle görünce durur, loga
`!!! GÜVENLİK ALARMI` yazar, RViz'de kırmızı `ALARM #n` işareti koyar, olayı
`~/.ros/fsb_security/` altına kaydeder ve 5 sim-s sonra devriyeye devam eder. İşaretli görüntü:
`ros2 run rqt_image_view rqt_image_view /perception/image_annotated`.

**İsteğe bağlı ölçüm sekmeleri** (ön planda; Ctrl+C → özet):
```bash
# Sekme 3 — gerçek yoldan kapsama (sekme 2'de coverage:=true verdiyseniz gerekmez)
ros2 run fsb_patrol coverage_monitor --ros-args -p use_sim_time:=true \
    -p map_yaml:=$PWD/src/fsb_navigation/maps/factory.yaml -p png:=$HOME/coverage.png
# Sekme 4 — çarpışma / açıklık (Gazebo gerçek pozu, ROS değil gz-transport)
ros2 run fsb_gazebo clearance_monitor.py --report 300
# Sekme 5 — lokalizasyon sıçraması ve kurtarma davranışları
ros2 run fsb_gazebo localization_monitor.py --report 300 --ros-args -p use_sim_time:=true
ros2 run fsb_gazebo recovery_counter.py --ros-args -p use_sim_time:=true
```

Eski 4 noktalı devriye (fallback), sekme 2 yerine:
```bash
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10
```

Notlar:
- Robot (0,0,0)'da doğar ve AMCL başlangıç pozu da budur (`set_initial_pose: true`); *2D Pose Estimate* gerekmez.
  Yanlış pozdan yakınsamayı izlemek isterseniz RViz'de *2D Pose Estimate* ile bilerek hatalı poz verin.
- Bir tur ~13–15 sim-dk sürer (30 hedef, ~145 m; Aşama 5b ölçümü 781–927 sim-s). GUI açıkken RTF düşerse
  duvar saatinde daha uzun sürer (sim süresi değişmez, `use_sim_time`).
- Kapatma: önce devriye sekmesinde Ctrl+C (hedef iptal edilir, robot durur), sonra launch sekmesinde Ctrl+C
  (Gazebo, RViz ve Nav2 birlikte kapanır).
- WSLg'de Gazebo GUI + RViz birlikte CPU'yu zorlar. Pencere açılmaz/çökerse: `export LIBGL_ALWAYS_SOFTWARE=1`
  (yavaş ama çalışır). Gazebo GUI gerekmiyorsa `headless:=true` ile yalnızca RViz'den izleyin.
- Waypoint'ler harita değişince yeniden üretilir: `fsb_patrol/README.md` → `patrol_planner`.

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
