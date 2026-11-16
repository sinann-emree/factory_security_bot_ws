# fsb_perception

**Amaç:** YOLOv8-pose (ultralytics) ile kamera görüntüsünde insan tespiti, duruş sınıflandırma
(yatan / ayakta) ve yalnızca **yerde yatan insan** için güvenlik alarmı.

**Durum:** Aşama 6b tamam. `yolo_node` simülasyonda doğrulandı; aynı düğüm gerçek kamerada da
çalışır (yalnızca `image_topic` / `use_sim_time` değişir).

## Python ortamı (önemli)
`~/.local` içinde pip ile kurulmuş bir OpenCV (`opencv-python` 5.x) varsa sistemdeki apt OpenCV
4.6'yı gölgeler ve `cv_bridge` bozulur (`cv2_to_imgmsg` → `KeyError: 16`; cv_bridge OpenCV 4.6 ile
derlendi). Çözüm `~/.local`'a dokunmaz:

1. `~/fsb_venv` (`--system-site-packages`): torch + ultralytics yalnızca burada, **opencv-python yok**.
2. Düğüm `PYTHONNOUSERSITE=1` ile başlar → `~/.local` yola girmez, `cv2` = apt OpenCV 4.6.
   (Yalnızca venv yetmez: `--system-site-packages` venv'inde `~/.local` hâlâ sistem paketlerinden
   önce gelir.)

`perception.launch.py` ikisini de kendisi ayarlar (venv yorumlayıcısı + ortam değişkenleri); elle
`activate` gerekmez. `colcon build` venv etkin **değilken** çalıştırılır.

```bash
# python3-venv (ensurepip) kurulu değilse --without-pip + sistemdeki pip kullanılır
python3 -m venv ~/fsb_venv --system-site-packages --without-pip
export PYTHONNOUSERSITE=1     # pip ~/.local'daki paketleri "kurulu" saymasın
~/fsb_venv/bin/python3 -m pip install torch torchvision polars nvidia-ml-py ultralytics-thop ultralytics-platform
~/fsb_venv/bin/python3 -m pip install --no-deps ultralytics   # --no-deps: opencv-python gelmesin
~/fsb_venv/bin/python3 -m pip uninstall -y setuptools          # torch'un çektiği ≥80 sürümü colcon ile uyumsuz
# doğrulama: cv2 4.6.0 /usr/lib/python3/dist-packages/..., hata yok
source /opt/ros/jazzy/setup.bash
~/fsb_venv/bin/python3 -c "import cv2, ultralytics; from cv_bridge import CvBridge; print(cv2.__version__, cv2.__file__)"
```
Doğrulanan sürümler: torch 2.15.0+cu132, ultralytics 8.4.201, apt OpenCV 4.6.0, numpy 1.26.4 (apt).
ultralytics `opencv-python>=4.7` ister; 4.6 ile çıkarım ve çizim sorunsuz çalışıyor (`pip check`
bu yüzden uyarır, beklenen durum). Düğüm açılışta `cv2` sürümünü denetler; 4.x değilse açıklayıcı
hata verip çıkar.

Model: `~/yolov8n-pose.pt` (6.8 MB, COCO insan + 17 anahtar nokta). Dosya yoksa düğüm hata verir,
kendiliğinden indirmez. İndirmek için:
`~/fsb_venv/bin/python3 -c "from ultralytics import YOLO; YOLO('yolov8n-pose.pt')"`.
Pose olmayan bir model (`~/yolov8n.pt`) verilirse düğüm yalnızca kutu oranı ipucuyla çalışır.

## Duruş sınıflandırma (`fsb_perception/posture.py`)
Her insan tespiti `lying` / `upright` / `unknown` olarak sınıflanır (ROS'tan bağımsız saf işlev):

| Sıra | Koşul | Karar (ipucu) |
|---|---|---|
| 1 | omuz + kalça anahtar noktaları ≥ `kp_conf` ve gövde ekseni (omuz ortası → kalça ortası) düşeyden ≥ `torso_lying_deg` | yatan (`torso`) |
| 2 | gövde geçerli, açı ≤ `torso_upright_deg` ve kutu oranı < `aspect_lying` | ayakta |
| 3 | kutu genişlik/yükseklik ≥ `aspect_lying` | yatan (`aspect`, yedek yol) |
| 4 | diğer | belirsiz |

**Zamansal doğrulama:** alarm, son `alert_window` (N) çıkarımın en az `alert_min_hits` (M) tanesinde
`alert_confidence` üstü yatan insan varsa verilir; koşul sürdükçe `alert_period` s'de bir yinelenir.
Ayakta/yürüyen insan ve diğer nesneler alarm üretmez.

## Launch
```bash
ros2 launch fsb_perception perception.launch.py use_sim_time:=true            # device otomatik
ros2 launch fsb_perception perception.launch.py use_sim_time:=true device:=cpu
# bringup ile birlikte (yerde yatan insanlı dünya + devriye + olay listesi):
ros2 launch fsb_bringup bringup.launch.py localization:=amcl patrol:=true perception:=true \
    world:=$(ros2 pkg prefix fsb_gazebo)/share/fsb_gazebo/worlds/factory_lying.sdf
```
Argümanlar: `use_sim_time`, `venv` (`~/fsb_venv`), `params_file` (`config/yolo.yaml`),
`model_path` (`~/yolov8n-pose.pt`), `device` (`''` = CUDA varsa `cuda:0`, yoksa `cpu`),
`image_topic` (`/camera/image_raw`), `image_transport` (`raw` | `compressed`), `omp_threads` (`2`).

### Sunucu mimarisi (YOLO başka makinede)
`yolo_node` robot sürücülerine, TF'e ya da Nav2'ye bağlı değildir; tek girdisi görüntü topic'idir.
```bash
# robot (ya da sim makinesi): ham görüntüyü JPEG olarak da yayınla
ros2 launch fsb_perception image_compress.launch.py use_sim_time:=true
# sunucu (aynı ROS_DOMAIN_ID; yalnızca fsb_perception + ~/fsb_venv gerekir)
ros2 launch fsb_perception perception.launch.py use_sim_time:=true image_transport:=compressed
# tek makinede denemek için: bringup ... perception:=true image_transport:=compressed
```
`image_transport:=compressed` iken düğüm `<image_topic>/compressed` (`sensor_msgs/CompressedImage`)
dinler ve işaretli görüntüyü de `/perception/image_annotated/compressed` olarak verir.
Bağlantı koparsa robotun durması için robot tarafında `fsb_control` `cmd_vel_watchdog` kullanılır
(`bringup ... cmd_vel_watchdog:=true`).

## Topic'ler
| Topic | Tip | Not |
|---|---|---|
| `/camera/image_raw` ya da `/camera/image_raw/compressed` (abone) | `sensor_msgs/Image` / `CompressedImage` | sensör QoS, yalnızca son kare tutulur |
| `/perception/detections` | `vision_msgs/Detection2DArray` | her çıkarımda (boş da olsa); `class_id` = `person_lying` / `person_upright` / `person` (belirsiz); kutu piksel (merkez + boyut); başlık = kaynak karenin başlığı |
| `/perception/posture` | `std_msgs/String` (JSON) | kare başına: `stamp`, `people[]` = kutu, güven, duruş, ipucu, gövde açısı, kutu oranı, 17 anahtar nokta |
| `/perception/image_annotated` (+ `/compressed`) | `sensor_msgs/Image` (bgr8) / `CompressedImage` | yalnızca abone varken üretilir; yatan insan kırmızı kutu + iskelet |
| `/security/alerts` | `vision_msgs/Detection2D` | alarm olayı: yalnızca yatan insan, N karede M doğrulamasıyla |

Çıkarım ayrı bir iş parçacığında, en fazla `rate` Hz ve yalnızca yeni karede çalışır; görüntü
callback'i yalnızca son kareyi saklar (executor bloklanmaz, eski kare birikmez).
Alarma tepki (konumla, listeye ekle, çok açıdan kaydet, revisit) `fsb_patrol` `patrol_executor`'dadır.

## Parametreler (`config/yolo.yaml`)
| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `model_path` | `~/yolov8n-pose.pt` | ağırlık dosyası |
| `confidence` | `0.35` | yayınlanan tespitler için alt eşik |
| `device` | `''` | `''` otomatik, `cpu`, `cuda:0` |
| `target_classes` | `['person']` | sınıf adları; boş = tümü |
| `rate` | `5.0` | Hz, çıkarım üst sınırı |
| `imgsz` | `640` | çıkarım girdi boyutu |
| `cpu_threads` | `2` | `device=cpu` iken torch iş parçacığı |
| `image_topic` | `/camera/image_raw` | |
| `image_transport` | `raw` | `raw` \| `compressed` |
| `torso_lying_deg` | `45.0` | gövde ekseni düşeyden bu kadar yatıksa yatan |
| `torso_upright_deg` | `30.0` | bu kadar dikse (ve kutu oranı küçükse) ayakta |
| `aspect_lying` | `1.5` | kutu genişlik/yükseklik yedek eşiği |
| `kp_conf` | `0.5` | omuz/kalça anahtar noktası alt güveni |
| `alert_confidence` | `0.35` | alarm için yatan insan güven eşiği |
| `alert_window` / `alert_min_hits` | `10` / `4` | N / M |
| `alert_period` | `2.0` | s, alarm yineleme aralığı |

## Ölçüm: kamera yüksekliği ve alçak açıdan pose başarısı
Simülasyon düzeneği: 4 Fuel insan modeli × 3 yatış (sırtüstü, yüzüstü, yan) × 4 mesafe
(1.5 / 2.5 / 4 / 6 m) × 5 gövde yönü, her poz 2 kare; ayrıca ayakta insanlar ve kutular. Aynı
sahne 4 kamera yüksekliğinden görüntülendi (toplam 2464 kare; robotun kamerası 4.7 cm'de).

| Kamera yüksekliği | yatan insanı bulan kare (güven ≥ 0.5) | "yatan" kararı (kural, güven ≥ 0.35) | poz başına (2 karede ≥ 1) | ayakta insanda yanlış "yatan" | kutuda yanlış "yatan" |
|---|---|---|---|---|---|
| 4.7 cm (mevcut) | %55.2 | %55.0 | %51.8 | 0 / 128 | 0 / 48 |
| 15 cm | %60.7 | — | — | 0 / 128 | 0 / 48 |
| 30 cm | %59.5 | %58.6 | — | 0 / 128 | 0 / 48 |
| 60 cm | %67.5 | %66.8 | %63.6 | 0 / 128 | 0 / 48 |

- 4.7 cm'de kararı karelerin %38.6'sında gövde ekseni, %12.7'sinde kutu oranı (yedek yol) verdi;
  %48.6'sında karar çıkmadı. Yedek yol kaldırılırsa yakalama ~13 puan düşer.
- Zayıf durumlar (4.7 cm): gövde bakış doğrultusunda uzanıyorsa (baş ya da ayak kameraya dönük:
  %30–33; yandan %53–61), 6 m'de (%25; 2.5 m'de %69), çok yakında gövde kadraja sığmıyorsa
  (1.5 m: %37), yan yatışta (%32; sırtüstü %57). Modele göre: Standing person %17,
  Walking person %51, Casual female %57, Rescue Randy %68.
- Özellik dağılımı: yatan insanda kutu oranı medyan 2.75 (p5 0.87), gövde açısı medyan 83°;
  ayakta insanda kutu oranı medyan 0.47 (p95 1.17), gövde açısı ≤ 4°. Eşikler bu aralığın ortasında.

**Öneri:** Tek karede başarı 4.7 cm'de ~%55; kamera 60 cm'e çıkarılırsa ~%67 (2.5–4 m'de %62 → %71).
Gerçek robotta kamerayı ≥ 30–60 cm yüksekliğe, hafif aşağı bakacak şekilde monte etmek önerilir.
Mevcut yükseklikte sistem tek kareye değil, yaklaşırken biriken karelere (N karede M) ve çok açılı
kayda dayanır; kutu oranı yedek yolu açık tutulmalıdır. Bu sayılar Fuel modelleriyle simülasyon
ölçümüdür; gerçek insan/giysi/ışık için doğrulanmadı.

## Doğrulama
```bash
ros2 topic hz /perception/detections          # ~5 Hz
ros2 topic echo /perception/posture --once
ros2 topic echo /security/alerts --once
ros2 run rqt_image_view rqt_image_view /perception/image_annotated
ros2 run fsb_gazebo posture_monitor.py        # duruşa göre kare sayıları + alarm sayısı (Ctrl+C → özet)
ros2 run fsb_gazebo perf_monitor.py --duration 240   # RTF + süreç gruplarına göre CPU
colcon test --packages-select fsb_perception  # posture.py birim testleri
```
