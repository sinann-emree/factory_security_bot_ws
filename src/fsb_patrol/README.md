# fsb_patrol

**Amaç:** Haritadan otomatik üretilen, dünyadan bağımsız, tam kapsamlı devriye.
(Aşama 6: `/security/alerts` olaylarına tepki — dur, uyar, kaydet.)

**Durum:** Aşama 5b — `patrol_planner` + `patrol_executor` + `coverage_monitor`. Eski 4 noktalı
`patrol_node` / `config/patrol.yaml` yedek (fallback) olarak duruyor (aşağıda).

Kodda dünya koordinatı yoktur. Girdi: harita yaml/pgm + `config/robot_params.yaml`. Yeni harita
geldiğinde yalnızca planlayıcı yeniden çalıştırılır.

## Kütüphaneler (Jazzy / Ubuntu 24.04)
Yalnızca apt/rosdep paketleri: `python3-numpy` 1.26.4, `python3-scipy` 1.11.4, `python3-yaml`,
`python3-matplotlib` 3.6.3 (PNG). scikit-image (kurulu değil), networkx (yalnızca pip --user) ve
OpenCV (pip --user 5.0.0 apt 4.6'yı gölgeliyor) bilinçli olarak KULLANILMADI; iskelet (Zhang–Suen),
budama, TSP saf numpy/scipy ile yazıldı. Testler `PYTHONNOUSERSITE=1` ile de geçer.

## 1. `patrol_planner` — harita → waypoint yaml
1. Serbestle çevrili, yalnızca bilinmeyen küçük adacıklar (< `speckle_area`) serbest sayılır.
2. Açıklık ≥ çevrel yarıçap + güvenlik payı olan hücreler = geçilebilir; dar boğazlar elenir,
   en büyük bileşen alınır.
3. Medial axis iskeleti (Zhang–Suen), `prune_length` altı dallar budanır; kavşak + uç düğümleri +
   `sample_step` aralıklı iskelet noktaları = adaylar.
4. Koridor segmentleri (dar bölgede, kavşak dairesi dışında, ≥ `corridor_min_length`): her birinin
   içine bir waypoint zorunlu → raf aralarına/koridorlara fiilen girilir.
5. Aday × `yaw_sectors` yön için kamera FOV'u + menzil içinde ışın izleme; tembel greedy set cover
   ≥ `target_coverage`, ardından gereksizleri atma. Yaw = kapsanmamış alana bakan sektör.
6. Doğrulama: InflationLayer eşdeğeri maliyet ≤ `max_waypoint_cost`, ayrıca canlı
   `/global_costmap/costmap` kontrolü (Nav2 varsa).
7. Maliyet matrisi = Nav2 `compute_path_to_pose` gerçek yol uzunlukları (`--cost nav2`) veya
   ızgara Dijkstra (`--cost grid`, Nav2'siz). TSP: n ≤ 12 Held–Karp, aksi çok başlangıçlı NN +
   2-opt + Or-opt.

```bash
# Nav2'siz arka uç (Gazebo gerekmez; sim çalışıyorsa gerek yok)
ros2 launch fsb_patrol planner_backend.launch.py map:=<harita.yaml>
ros2 run fsb_patrol patrol_planner --cost nav2 --map <harita.yaml> \
    --out src/fsb_patrol/config/waypoints_factory.yaml --png /tmp/waypoints.png
# Hızlı, Nav2'siz:
ros2 run fsb_patrol patrol_planner --cost grid --map <harita.yaml> --out <wp.yaml>
```
Kapsama hedefin altındaysa çıkış kodu 2. Çıktı yaml: harita sha256, özet (kapsama toplam/dar/
geniş, tur uzunluğu, koridor segmentleri), her waypoint için x, y, yaw, tür, açıklık, maliyet ve
`path_lengths_m` matrisi.

## 2. `patrol_executor` — Nav2 ile gezme
```bash
ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true            # idleness, sonsuz
ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true mode:=sequential loops:=5
ros2 launch fsb_patrol patrol_executor.launch.py use_sim_time:=true mode:=random coverage:=true
```
| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `mode` | `idleness` | `idleness` (skor = ağırlık·bekleme − yol_süresi), `sequential` (TSP sırası), `random` |
| `loops` | `0` | tur sayısı, 0 = sonsuz; tur = tüm hedefler ziyaret edildi veya atlandı |
| `waypoints_file` | `config/waypoints_factory.yaml` | planlayıcı çıktısı |
| `priority_file` | `config/priority_points_factory.yaml` | dünyaya özel öncelikli noktalar (ağırlıklı) |
| `goal_timeout` | `180.0` | sim-s; aşılırsa hedef iptal edilir ve atlanır |
| `nominal_speed` | `0.2` | m/s, idleness skorunda yol süresi tahmini |

Ulaşılamayan / zaman aşan waypoint `✗ ATLANDI ... tur devam ediyor` ile loglanır, tur iptal
olmaz. RViz için `/patrol/waypoints` (`visualization_msgs/MarkerArray`, transient-local) yayınlar:
her hedef için yön oku + etiket; turuncu = şu anki hedef, yeşil = bu turda ziyaret, kırmızı = atlandı,
mavi = bekliyor, mor = bekleyen öncelikli nokta (`fsb_bringup/rviz/nav.rviz`'de hazır). Her tur sonunda `Tur k TAMAM (sim/duvar)` özeti, en sonda `DEVRİYE BİTTİ`.

## 3. `coverage_monitor` — gerçek yoldan kapsama
TF `map→camera_link` pozlarından planlayıcıyla AYNI görüş modeliyle (FOV + menzil + ışın izleme)
gerçekten görülen serbest alan yüzdesini, dar bölge kapsamasını ve girilen koridor segmentlerini
ölçer; SIGINT/SIGTERM'de `SON ÖZET` ve isteğe bağlı PNG. `patrol_executor.launch.py coverage:=true`
ile birlikte de başlar.
```bash
ros2 run fsb_patrol coverage_monitor --ros-args -p use_sim_time:=true \
    -p map_yaml:=<harita.yaml> -p png:=/tmp/coverage.png
```

## 4. Genelleme kontrolü — `map_variants`
Haritanın döndürülmüş (90°, 30°), aynalanmış, kırpılmış ve birleşik (ayna + 17°) varyantlarında
(yalnızca pgm, Gazebo/Nav2'siz) planlayıcıyı çalıştırır; kapsama ≥ hedef, tüm waypoint'ler geçerli,
yollar sonlu, tüm koridor segmentlerine giriliyor mu kontrol eder. Aynı kontrol
`test/test_planner_variants.py` ile `colcon test`'te koşar.
```bash
ros2 run fsb_patrol map_variants --map src/fsb_navigation/maps/factory.yaml --out-dir /tmp/variants
```

## `config/robot_params.yaml`
Robot (çevrel/iç yarıçap, güvenlik payı), sensör (LiDAR menzili, kamera hfov, görüş menzili),
costmap (inflation, maliyet eşiği) ve planlayıcı ayarları. Her değerin kaynağı yorumda yazılıdır.

## Fallback: `patrol_node` (`fsb_patrol/patrol_node.py`)
Her tur için bir `nav2_msgs/action/FollowWaypoints` hedefi gönderir (`number_of_loops=0`), sonuç gelince
bir sonraki turu başlatır. Böylece her tur ayrı loglanır (süre, kaçırılan waypoint'ler, hata kodu).
Bir tur başarısız olsa da devriye durmaz; sonraki tur denenir. Bitince `DEVRİYE BİTTİ: x/y tur başarılı` yazar.

## Launch
Nav2 çalışırken (`ros2 launch fsb_bringup bringup.launch.py localization:=amcl`):
```bash
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true            # sonsuz
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10  # 10 tur
```

## Parametreler (`config/patrol.yaml`)
| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `waypoints` | 4 nokta (aşağıda) | düz liste `[x, y, yaw, ...]` (m, m, rad), en az 3 nokta |
| `frame_id` | `map` | waypoint çerçevesi |
| `loops` | `0` | tur sayısı, 0 = sonsuz (launch argümanı `loops` ile ezilir) |
| `action_name` | `follow_waypoints` | Nav2 waypoint_follower action adı |

Varsayılan rota (factory.sdf engellerinden ≥ 1 m boşluklu):
W1 (4.5, 0) → W2 (3, 3) → W3 (-3, -3) → W4 (-4.5, 0) → W1 …
