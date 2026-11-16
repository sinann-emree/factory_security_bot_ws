# fsb_perception

**Amaç:** YOLOv8 (ultralytics) ile kamera görüntüsünde insan tespiti ve güvenlik alarmı.

**Durum:** Aşama 6 tamam. `yolo_node` simülasyonda doğrulandı; aynı düğüm gerçek kamerada da
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

Model: `~/yolov8n.pt` (6.5 MB, COCO; `person` = sınıf 0). Dosya yoksa düğüm hata verir, kendiliğinden
indirmez. İndirmek için: `~/fsb_venv/bin/python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"`.

## Launch
```bash
ros2 launch fsb_perception perception.launch.py use_sim_time:=true            # device otomatik
ros2 launch fsb_perception perception.launch.py use_sim_time:=true device:=cpu
# bringup ile birlikte (insan modelli dünya + devriye):
ros2 launch fsb_bringup bringup.launch.py localization:=amcl patrol:=true perception:=true \
    world:=$(ros2 pkg prefix fsb_gazebo)/share/fsb_gazebo/worlds/factory_person.sdf
```
Argümanlar: `use_sim_time`, `venv` (`~/fsb_venv`), `params_file` (`config/yolo.yaml`),
`model_path` (`~/yolov8n.pt`), `device` (`''` = CUDA varsa `cuda:0`, yoksa `cpu`).

## Topic'ler
| Topic | Tip | Not |
|---|---|---|
| `/camera/image_raw` (abone) | `sensor_msgs/Image` | sensör QoS, yalnızca son kare tutulur |
| `/perception/detections` | `vision_msgs/Detection2DArray` | her çıkarımda (boş da olsa); `results[0].hypothesis.class_id` = sınıf adı, `score` = güven; kutu piksel (merkez + boyut); başlık = kaynak karenin başlığı |
| `/perception/image_annotated` | `sensor_msgs/Image` (bgr8) | yalnızca abone varken üretilir; alarm eşiği üstü kutular turuncu |
| `/security/alerts` | `vision_msgs/Detection2D` | alarm olayı: `alert_classes` sınıfı art arda `alert_min_frames` çıkarımda `alert_confidence` üstü; hedef görünürken `alert_period` s'de bir yinelenir |

Çıkarım ayrı bir iş parçacığında, en fazla `rate` Hz ve yalnızca yeni karede çalışır; görüntü
callback'i yalnızca son kareyi saklar (executor bloklanmaz, eski kare birikmez).
Alarma tepki (dur, uyar, kaydet, devam) `fsb_patrol` `patrol_executor`'dadır.

## Parametreler (`config/yolo.yaml`)
| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `model_path` | `~/yolov8n.pt` | ağırlık dosyası |
| `confidence` | `0.5` | yayınlanan tespitler için alt eşik |
| `device` | `''` | `''` otomatik, `cpu`, `cuda:0` |
| `target_classes` | `['person']` | sınıf adları; boş = tümü |
| `rate` | `5.0` | Hz, çıkarım üst sınırı |
| `imgsz` | `640` | çıkarım girdi boyutu |
| `cpu_threads` | `2` | `device=cpu` iken torch iş parçacığı |
| `image_topic` | `/camera/image_raw` | |
| `alert_classes` | `['person']` | alarm üreten sınıflar |
| `alert_confidence` | `0.8` | alarm güven eşiği |
| `alert_min_frames` | `3` | art arda eşik üstü çıkarım sayısı |
| `alert_period` | `2.0` | s, alarm yineleme aralığı |

## Doğrulama
```bash
ros2 topic hz /perception/detections          # ~5 Hz
ros2 topic echo /security/alerts --once
ros2 run rqt_image_view rqt_image_view /perception/image_annotated
ros2 run fsb_gazebo alert_response_monitor.py # güven dağılımı + alarm/durma ölçümü (Ctrl+C → özet)
ros2 run fsb_gazebo perf_monitor.py --duration 240   # RTF + süreç gruplarına göre CPU
```
