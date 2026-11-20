# fsb_navigation

**Amaç:** SLAM Toolbox (haritalama), AMCL (lokalizasyon) ve Nav2 yapılandırması, kaydedilmiş fabrika haritası.

**Durum:** Aşama 5 tamam.

## Dizinler
- `config/slam_toolbox_mapping.yaml` — online async haritalama (base_frame `base_footprint`, `/scan`)
- `config/nav2_params.yaml` — AMCL, map_server, bt_navigator, controller (RegulatedPurePursuit), planner (SmacPlanner2D; Aşama 5b'de NavFn'den geçildi, gerekçe yaml'da),
  costmap'ler, behavior_server, waypoint_follower, velocity_smoother, collision_monitor.
  Costmap engel kaynakları: `scan` (LiDAR) + `keepout` (`/security/keepout`, `PointCloud2`, `map`
  çerçevesi; `fsb_patrol` olay listesindeki yatan kişilerin çevresi — yayıncı yokken etkisiz)
- `maps/factory.yaml|pgm` — SLAM ile çıkarılmış harita (0.05 m/hücre)
- `launch/slam.launch.py` — slam_toolbox (lifecycle) + lifecycle_manager
- `launch/navigation.launch.py` — map_server + AMCL + Nav2 (nav2_bringup'ın opennav_docking'i yok)

## Kullanım (bringup üzerinden)
```bash
# Haritalama (map→odom: slam_toolbox)
ros2 launch fsb_bringup bringup.launch.py headless:=true localization:=slam
ros2 run fsb_gazebo mapping_tour.py          # veya teleop
ros2 run nav2_map_server map_saver_cli -f src/fsb_navigation/maps/factory --ros-args -p use_sim_time:=true

# Lokalizasyon + navigasyon (map→odom: AMCL)
ros2 launch fsb_bringup bringup.launch.py headless:=true localization:=amcl [map:=<yaml>]
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10
```

## TF ve cmd_vel
- `map→odom`: haritalamada YALNIZCA slam_toolbox, lokalizasyonda YALNIZCA AMCL (`localization` argümanı ikisini birlikte açmaz).
- `odom→base_footprint`: YALNIZCA EKF.
- cmd_vel zinciri, hepsi `geometry_msgs/msg/TwistStamped` (`ros2 topic info -v` ile doğrulandı):
  `controller_server`, `behavior_server` → `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel_smoothed`
  → `collision_monitor` → `/cmd_vel` → `diff_drive_controller`.
  Nav2 1.3.13'te `enable_stamped_cmd_vel` varsayılanı **false**; zincirdeki her düğümde `true`.

## Önemli parametreler ve gerekçeleri
| Parametre | Değer | Gerekçe |
|---|---|---|
| footprint | x∈[-0.204, 0.070], y=±0.087 | fsb_properties.xacro ölçüleri + 1 cm pay (hesap yaml'da) |
| slam `max_laser_range`, amcl `laser_max_range` | 8.0 m (sensör 12 m) | Gazebo gpu_lidar teğet gelişte (80–85°) 4–12 m'de medyan +19…+37 cm uzun ölçer. 12 m ile haritada eğik hayalet duvarlar (dolu hücrelerin %50'si >10 cm), 8 m ile %0.04 |
| RPP `desired_linear_vel` | 0.2 m/s | diff_drive limitine (0.2 m/s) eşit |
| velocity_smoother `max_velocity` | [0.2, 0, 0.8] | diff_drive limitleri (0.2 m/s, 1.0 rad/s) altında/eşit |
| behavior_server dönüş | ≤ 0.8 rad/s | SLAM/lokalizasyon kalitesi için hızlı dönüşlerden kaçın |
| collision_monitor scan `min_height` | 0.05 | varsayılan 0.15 m, LiDAR yüksekliğinin (0.107 m) üstünde → tüm noktalar atılırdı |
| collision_monitor StopZone | footprint + 3 cm | |

## Aşama 5 doğrulama sonuçları (simülasyon)
- **Harita** (`map_quality_check.py`): dolu hücre → gerçek yüzey medyan 2.2 cm, p95 5.5 cm; %99.96 ≤ 10 cm
  (>10 cm: 3 hücre); global hizalama dθ=0.0°, dx=+2 cm, dy=0; duvar kalınlığı medyan 2 / maks 3 hücre
  (çift duvar yok); kapsama %99.1; poz grafiğinde 4 döngü kapatma kenarı.
- **Haritalama sırasında** (846 sim-s, 109 m): map→odom maks adım 6.8 cm / 0.40°, sıçrama 0;
  SLAM pozu ↔ gerçek konum ort. 3.7 cm, maks 8.6 cm.
- **AMCL** (`amcl_check.py`): 59.8 cm / 16.5° hatalı başlangıçtan 7.0 sim-s'de yakınsadı; son hata 1.2 cm / 0.5°,
  bulut σxy 70.7 → 25.7 cm. map→odom tek yayıncı: AMCL `tf_broadcast=false` iken 5 s'de 0 mesaj.
- **Dinamik engel** (`dynamic_obstacle_test.py`): SUCCEEDED, min. açıklık 38.8 cm, yanal sapma 0.76 m, kutu 0.00 cm oynadı.
