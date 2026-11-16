"""
yolo_node: kamera görüntüsünde YOLOv8 (ultralytics) ile nesne tespiti ve güvenlik alarmı.

Abone : image_topic (/camera/image_raw, sensor_msgs/Image)
Yayın : /perception/detections      vision_msgs/Detection2DArray  (her çıkarımda, boş da olsa)
        /perception/image_annotated sensor_msgs/Image bgr8        (yalnızca abone varken)
        /security/alerts            vision_msgs/Detection2D       (alarm olayı, aşağıya bakın)

İş parçacıkları: görüntü callback'i yalnızca son kareyi saklar (kopyasız, kilit altında); çıkarım
ayrı bir iş parçacığında, en fazla `rate` Hz (duvar saati) ve yalnızca yeni karede çalışır.
Böylece executor (callback'ler, parametre servisleri) çıkarım süresince bloklanmaz, eski kareler
birikmez.

Alarm: `alert_classes` içindeki bir sınıf art arda `alert_min_frames` çıkarımda
`alert_confidence` ve üzeri güvenle görülürse /security/alerts'e o karedeki en güvenli tespit
yayınlanır; hedef görünür kaldıkça en fazla `alert_period` saniyede bir yinelenir. Olayın tekilleş-
tirilmesi (bekleme süresi) tepki veren düğümün işidir (fsb_patrol patrol_executor).

Parametreler:
  model_path        ~/yolov8n.pt   ağırlık dosyası; yoksa düğüm hata verir (kendiliğinden indirmez)
  confidence        0.5            yayınlanan tespitler için alt güven eşiği
  device            ''             '' = CUDA varsa cuda:0, yoksa cpu; 'cpu' | 'cuda:0'
  target_classes    ['person']     yayınlanacak sınıf adları; [''] ya da boş = tüm sınıflar
  rate              5.0            Hz, çıkarım üst sınırı
  imgsz             640            çıkarım girdi boyutu (uzun kenar, piksel)
  cpu_threads       2              device=cpu iken torch iş parçacığı sayısı (Nav2'ye CPU kalsın;
                                   launch'taki OMP_NUM_THREADS üst sınırdır)
  image_topic       /camera/image_raw
  alert_classes     ['person'], alert_confidence 0.8, alert_min_frames 3, alert_period 2.0 (s)

Ortam: ultralytics + torch ~/fsb_venv içindedir ve düğüm PYTHONNOUSERSITE=1 ile başlatılmalıdır
(perception.launch.py bunu yapar). Aksi halde ~/.local'daki pip OpenCV 5 sistemdeki OpenCV 4.6'yı
gölgeler ve cv_bridge bozulur; düğüm bunu açılışta denetler.
"""

import os
import threading
import time
from typing import Dict, List, Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

BOX_COLOR = (0, 0, 255)      # BGR: hedef kutusu
ALERT_COLOR = (0, 165, 255)  # BGR: alarm eşiği üstü


class YoloNode(Node):

    def __init__(self) -> None:
        super().__init__('fsb_yolo_node')
        self.declare_parameter('model_path', '~/yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', '')
        self.declare_parameter('target_classes', ['person'])
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('cpu_threads', 2)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('alert_classes', ['person'])
        self.declare_parameter('alert_confidence', 0.8)
        self.declare_parameter('alert_min_frames', 3)
        self.declare_parameter('alert_period', 2.0)
        gp = self.get_parameter
        self.confidence = float(gp('confidence').value)
        self.period = 1.0 / max(float(gp('rate').value), 0.1)
        self.imgsz = int(gp('imgsz').value)
        self.alert_conf = float(gp('alert_confidence').value)
        self.alert_min_frames = int(gp('alert_min_frames').value)
        self.alert_period = float(gp('alert_period').value)

        cv_major = int(cv2.__version__.split('.')[0])
        if cv_major != 4:
            raise RuntimeError(
                f'cv2 {cv2.__version__} ({cv2.__file__}) yüklendi; cv_bridge sistemdeki OpenCV '
                '4.x ile derlendi. Düğümü perception.launch.py ile başlatın '
                '(PYTHONNOUSERSITE=1, ~/fsb_venv).')

        # Ağır içe aktarmalar burada: hata mesajı ROS logger'dan çıksın
        import torch
        from ultralytics import YOLO
        model_path = os.path.expanduser(str(gp('model_path').value))
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'model_path bulunamadı: {model_path}')
        device = str(gp('device').value) or ('cuda:0' if torch.cuda.is_available() else 'cpu')
        if device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warning(f'device={device} istendi ama CUDA yok; cpu kullanılıyor')
            device = 'cpu'
        # İş parçacığı sınırı: torch/OpenCV varsayılan olarak tüm çekirdeklere yayılır ve Nav2 ile
        # yarışır. (OpenMP havuzunun kendisi OMP_NUM_THREADS ile launch'ta sınırlanır.)
        torch.set_num_threads(max(1, int(gp('cpu_threads').value)) if device == 'cpu' else 1)
        cv2.setNumThreads(1)
        self.device = device
        self.model = YOLO(model_path)
        names: Dict[int, str] = dict(self.model.names)
        self.names = names

        def ids(param: str) -> List[int]:
            wanted = [str(c) for c in gp(param).value if str(c)]
            unknown = sorted(set(wanted) - set(names.values()))
            if unknown:
                raise ValueError(f'{param}: modelde olmayan sınıf(lar): {unknown}')
            return [i for i, n in names.items() if n in wanted]
        self.class_ids: Optional[List[int]] = ids('target_classes') or None
        self.alert_names = {names[i] for i in ids('alert_classes')}

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest: Optional[Image] = None
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._alert_streak = 0
        self._last_alert = -1e9
        self.stats = {'frames': 0, 'infer_ms': 0.0}

        self.det_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.img_pub = self.create_publisher(Image, '/perception/image_annotated', 1)
        self.alert_pub = self.create_publisher(Detection2D, '/security/alerts', 10)
        self.create_subscription(Image, str(gp('image_topic').value), self._on_image,
                                 qos_profile_sensor_data)

        # Isınma: ilk çıkarımın (CUDA başlatma, katman birleştirme) gecikmesi akışa yansımasın
        t0 = time.perf_counter()
        self._infer(np.zeros((self.imgsz, self.imgsz, 3), np.uint8))
        self.get_logger().info(
            f'YOLO hazır: {os.path.basename(model_path)}, device={device}, '
            f'sınıflar={[names[i] for i in self.class_ids] if self.class_ids else "tümü"}, '
            f'conf≥{self.confidence}, {1.0 / self.period:.1f} Hz, imgsz={self.imgsz}, '
            f'alarm={sorted(self.alert_names)}≥{self.alert_conf} × {self.alert_min_frames} kare '
            f'(ısınma {time.perf_counter() - t0:.1f} s, cv2 {cv2.__version__})')
        self._worker = threading.Thread(target=self._run, name='yolo_infer', daemon=True)
        self._worker.start()
        self.create_timer(30.0, self._report)

    # ------------------------------------------------------------------ executor tarafı
    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest = msg
        self._new_frame.set()

    def _report(self) -> None:
        n, ms = self.stats['frames'], self.stats['infer_ms']
        self.stats = {'frames': 0, 'infer_ms': 0.0}
        if n:
            self.get_logger().info(f'son 30 s: {n} çıkarım ({n / 30.0:.1f} Hz), '
                                   f'ort. {ms / n:.1f} ms/kare')

    # ------------------------------------------------------------------ çıkarım iş parçacığı
    def _infer(self, bgr: np.ndarray):
        return self.model.predict(bgr, device=self.device, conf=self.confidence,
                                  classes=self.class_ids, imgsz=self.imgsz, verbose=False)[0]

    def _run(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            if not self._new_frame.wait(timeout=0.2):
                continue
            delay = next_t - time.monotonic()
            if delay > 0 and self._stop.wait(delay):
                break
            self._new_frame.clear()
            with self._lock:
                msg = self._latest
            if msg is None:
                continue
            next_t = max(next_t + self.period, time.monotonic())
            try:
                self._process(msg)
            except Exception as e:  # noqa: BLE001 - tek kare hatası düğümü düşürmesin
                if rclpy.ok():
                    self.get_logger().error(f'çıkarım hatası: {e!r}', throttle_duration_sec=5.0)

    def _process(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t0 = time.perf_counter()
        res = self._infer(bgr)
        self.stats['infer_ms'] += 1000.0 * (time.perf_counter() - t0)
        self.stats['frames'] += 1

        out = Detection2DArray()
        out.header = msg.header
        best_alert: Optional[Detection2D] = None
        boxes = res.boxes
        xywh = boxes.xywh.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy().astype(int)
        for (cx, cy, w, h), conf, cls in zip(xywh, confs, clss):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x, det.bbox.center.position.y = float(cx), float(cy)
            det.bbox.size_x, det.bbox.size_y = float(w), float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names[int(cls)]
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            out.detections.append(det)
            if hyp.hypothesis.class_id in self.alert_names and conf >= self.alert_conf and (
                    best_alert is None or conf > best_alert.results[0].hypothesis.score):
                best_alert = det
        self.det_pub.publish(out)

        self._alert_streak = self._alert_streak + 1 if best_alert is not None else 0
        now = time.monotonic()
        if best_alert is not None and self._alert_streak >= self.alert_min_frames and \
                now - self._last_alert >= self.alert_period:
            self._last_alert = now
            self.alert_pub.publish(best_alert)
            h = best_alert.results[0].hypothesis
            self.get_logger().warning(
                f'ALARM: {h.class_id} güven {h.score:.2f} ({self._alert_streak} kare üst üste)',
                throttle_duration_sec=10.0)

        if self.img_pub.get_subscription_count() > 0:
            for det in out.detections:
                h = det.results[0].hypothesis
                x0 = int(det.bbox.center.position.x - det.bbox.size_x / 2)
                y0 = int(det.bbox.center.position.y - det.bbox.size_y / 2)
                x1, y1 = int(x0 + det.bbox.size_x), int(y0 + det.bbox.size_y)
                color = ALERT_COLOR if (h.class_id in self.alert_names
                                        and h.score >= self.alert_conf) else BOX_COLOR
                cv2.rectangle(bgr, (x0, y0), (x1, y1), color, 2)
                cv2.putText(bgr, f'{h.class_id} {h.score:.2f}', (x0, max(y0 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            ann = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            ann.header = msg.header
            self.img_pub.publish(ann)

    def shutdown(self) -> None:
        self._stop.set()
        self._worker.join(timeout=2.0)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():   # kapanış sırasında (SIGINT) geçersiz bağlam hatası yutulur
            raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
