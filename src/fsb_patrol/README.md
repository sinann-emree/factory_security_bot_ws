# fsb_patrol

**Amaç:** Haritadan otomatik üretilen, dünyadan bağımsız, tam kapsamlı devriye.
(Aşama 6b: yerde yatan insan için olay listesi — konumla, kaydet, revisit, çözüldü.)

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

### Yatan insan: olay listesi (watchlist), kayıt ve revisit
Alarm **yalnızca yerde yatan insan** içindir (ayakta/yürüyen insan ve hareketli nesne kapsam dışı).
`fsb_perception` `yolo_node` yatan insanı son N çıkarımın M'sinde görünce `/security/alerts`
(`vision_msgs/Detection2D`) yayınlar. `patrol_executor` (kod: `incident_manager.py`,
`incident_geometry.py`, `watchlist.py`):

1. **Konum** — kişinin `map` konumu hesaplanır: tespit kutusunun yatay açı aralığına düşen LiDAR
   noktalarından en yakın bağlantılı küme (komşu aralığı ≤ 0.3 m); konum kümenin ortasıdır, küme
   noktaları gövdenin görünen kısmı (`extent`) olarak saklanır (`method: lidar`). LiDAR noktası yoksa
   kutunun alt kenarından zemin düzlemi ışını (`method: ground_ray`; kamera yere çok yakın olduğu
   için menzili güvenilmez, yalnızca yedek).
2. **Birleştir** — konum etkin bir kaydın merkezine **ya da bilinen gövde noktalarına**
   `merge_radius` (1 m) içindeyse aynı kişidir: yeni kayıt açılmaz, bildirim yinelenmez, devriye
   durmaz (yalnızca `last_seen` güncellenir). Gövde ~1.7 m uzunluğunda olduğundan yalnızca merkeze
   bakmak aynı kişinin iki ucunu ayrı kişi sayıyordu (ölçüm: 9 kişide 3 çift kayıt). Zemin ışını
   konumlarında menzil yerine bakış doğrultusu kullanılır: doğrultu üzerinde bilinen kişi varsa
   odur. Böylece listedeki kişi sıradan devriyede her görüldüğünde yeniden alarm üretmez.
   Uzak tespit (`add_max_range`, 6.5 m ötesi) yeni kayıt açmaz: o uzaklıkta kutu–LiDAR eşlemesi
   güvenilmezdir (ölçüm: 10 m'den 2.7 m hatalı konum); devriye yaklaşınca kişi yeniden görülür.
   Kayıtta hiçbir açıdan doğrulanamamış bir kaydın `unconfirmed_merge_radius` (2 m) yakınındaki
   yeni tespit, yeni kayıt açmak yerine o kaydı yeni konuma taşır ve kaydı yineler.
3. **Ekle + bildir (bir kez)** — yeni kişi listeye eklenir, `!!! YATAN İNSAN #n` WARN logu, RViz'de
   kırmızı küre + `YATAN #n (doğrulama k)` etiketi (`/patrol/waypoints`, `alert` ad alanı).
4. **Kayıt görevi** — etkin devriye hedefi iptal edilir; kişinin çevresinde `inspect_radius`
   uzaklıkta, birbirinden ≥ 50° ayrık `inspect_views` bakış noktası global costmap'ten seçilir
   (maliyeti düşük ve LiDAR'a göre kişiye görüş hattı açık hücreler), her birinde `observe_time`
   boyunca kare sayılır, işaretli kamera karesi (`wl<id>_inspect<k>_<zaman>.jpg`) ve duruş kaydı
   (kutu, güven, gövde açısı, anahtar noktalar) saklanır. Kayıt sırasında toplanan konumların
   medyanıyla kişinin konumu inceltilir. Ulaşılamayan bakış noktası atlanır, sıradaki denenir.
5. **Devam** — devriye kaldığı hedeften sürer (hedef atlanmış sayılmaz).
   **Yasak bölge:** kayıt etkin olduğu sürece kişinin çevresi (`keepout_radius`, 1 m) Nav2
   costmap'lerine engel olarak bildirilir (`/security/keepout`, `PointCloud2`, 1 Hz;
   `fsb_navigation` `nav2_params.yaml` → `obstacle_layer.keepout`). Gerekçe: LiDAR düzlemi
   (zeminden ~11 cm) yatan gövdenin yalnızca bir kısmını görür; yasak bölge olmadan bir denemede
   robot, kayıt sırasında kişi ile raf arasından geçerken gövdeye sıkıştı. Bölge içinde kalan
   devriye noktaları kayıt çözülene dek atlanır.
6. **Revisit** — `revisit_interval` sim-s sonra (bir sonraki hedef arasında) konumu gören bir bakış
   noktasına gidilir:
   - **hâlâ yerde** (≥ `observe_min_hits` karede konumun yakınında yatan insan) → bildirim yenilenir
     (`!!! YATAN İNSAN #n HÂLÂ YERDE`), `confirmations` ve `notifications` artar, yeni revisit kurulur;
   - **yok** → kayıt **yalnızca** şu üçü birden doğrulanırsa listeden çıkarılır ve `ÇÖZÜLDÜ` loglanır:
     konum kamera görüş alanında ve `view_max_range` içinde, LiDAR'a göre görüş hattı açık (araya
     engel girmemiş), konumda LiDAR dönüşü yok (zemin boş);
   - **doğrulanamadı** (bakış noktasına ulaşılamadı, konum FOV dışında, araya engel girdi ya da
     konumda hâlâ bir şey var ama yatan insan olarak tanınmadı) → kayıt **silinmez**; sıradaki bakış
     noktası denenir, hepsi tükenirse `revisit_retry` sim-s sonra yeniden denenir.

**Dosyalar** (`alert_log_dir`): `watchlist.yaml` (güncel liste; her değişiklikte atomik yazılır,
yeniden başlatmada etkin kayıtlar yüklenir), `events.jsonl` (`added`, `observation`, `revisit`,
`resolved` ... olayları: sim/duvar zamanı, bakış noktası, robot pozu, kare/isabet sayıları, görüş
doğrulaması), `wl<id>_*.jpg`.
Kayıt alanları: `id`, `x`, `y` (map), `status` (`active`/`resolved`), `first_seen_wall`,
`first_seen_sim`, `last_seen_sim`, `confirmations`, `notifications`, `images`, `observations`,
`revisits`, `next_revisit_sim`, `resolved_wall`, `extent`.

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `react_to_alerts` | `true` | `false` → alarmlar dinlenmez |
| `alert_topic` | `/security/alerts` | |
| `alert_max_age` | `2.0` | sim-s; bundan eski kareye ait (kuyrukta beklemiş) alarm yok sayılır |
| `alert_log_dir` | `~/.ros/fsb_security` | watchlist + olay günlüğü + kareler (launch argümanı da var) |
| `alert_image_topic` | `/perception/image_annotated` | `''` → kare kaydedilmez |
| `merge_radius` | `1.0` | m; aynı kişi sayma yarıçapı (launch argümanı da var) |
| `inspect_radius` | `2.0` | m; bakış noktalarının kişiye uzaklığı |
| `inspect_views` | `3` | kayıtta farklı açı sayısı (launch argümanı da var) |
| `observe_time` / `observe_settle` | `4.0` / `1.0` | sim-s; bakış noktasında sayım süresi / öncesinde bekleme |
| `observe_min_hits` | `3` | "görüldü / hâlâ yerde" için gereken yatan-insan karesi |
| `revisit_interval` | `300.0` | sim-s; kayıt ya da doğrulamadan sonraki revisit (launch argümanı da var) |
| `add_max_range` | `6.5` | m; bundan uzaktan konumlanan tespit yeni kayıt açmaz |
| `unconfirmed_merge_radius` | `2.0` | m; doğrulanamamış kaydı yeni tespitle taşıma yarıçapı |
| `keepout_radius` | `1.0` | m; etkin kaydın çevresindeki costmap yasak bölgesi (`0` = kapalı) |
| `keepout_topic` | `/security/keepout` | |
| `revisit_retry` | `60.0` | sim-s; görüş doğrulanamayınca yeniden deneme |
| `view_max_range` | `4.0` | m; "çözüldü" doğrulaması için kamera–konum üst uzaklığı |
| `scan_topic`, `camera_info_topic`, `costmap_topic`, `camera_frame`, `lidar_frame` | bkz. `incident_manager.py` | konum hesabı ve bakış noktası seçimi girdileri |

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
