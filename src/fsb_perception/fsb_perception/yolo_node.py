"""
yolo_node: YOLOv8-pose ile insan tespiti, duruş sınıflandırma ve yatan insan alarmı.

Abone : image_topic (sensor_msgs/Image)  ya da  image_topic + '/compressed'
        (sensor_msgs/CompressedImage; image_transport:=compressed)
Yayın : /perception/detections      vision_msgs/Detection2DArray  (her çıkarımda, boş da olsa)
            class_id: person_lying | person_upright | person (belirsiz) | <diğer sınıf adı>
        /perception/posture         std_msgs/String (JSON): kare başına kişi listesi — kutu, güven,
            duruş, gövde açısı, kutu oranı, 17 anahtar nokta (olay kaydı için)
        /perception/image_annotated sensor_msgs/Image bgr8        (yalnızca abone varken)
        /perception/image_annotated/compressed  CompressedImage   (yalnızca abone varken)
        /security/alerts            vision_msgs/Detection2D       (yatan insan alarmı)

Alarm YALNIZCA yatan insan içindir. Ayakta / yürüyen insan ve diğer nesneler tespit olarak
yayınlanır ama alarm üretmez. Zamansal doğrulama: son `alert_window` çıkarımın en az
`alert_min_hits` tanesinde güveni ≥ `alert_confidence` olan bir "yatan" tespiti varsa alarm verilir
(o karedeki en güvenli yatan tespit); koşul sürdükçe en fazla `alert_period` s'de bir yinelenir.
Kişinin haritadaki konumu, tekilleştirme ve tepki fsb_patrol patrol_executor'dadır.

Duruş: fsb_perception/posture.py (gövde ekseni + kutu oranı). Model pose modeli değilse
(anahtar nokta yok) yalnızca kutu oranı kullanılır.

İş parçacıkları: görüntü callback'i yalnızca son kareyi saklar; çıkarım ayrı bir iş parçacığında,
en fazla `rate` Hz ve yalnızca yeni karede çalışır (executor bloklanmaz, eski kare birikmez).

Başka makinede çalıştırma: düğüm yalnızca görüntü topic'ine abonedir; TF, tarama ya da robot
sürücüsü gerektirmez. Aynı ROS_DOMAIN_ID'deki bir sunucuda image_transport:=compressed ile
çalıştırılabilir (bkz. README).

Parametreler:
  model_path        ~/yolov8n-pose.pt  ağırlık dosyası; yoksa düğüm hata verir (indirmez)
  confidence        0.35           yayınlanan tespitler için alt güven eşiği
  device            ''             '' = CUDA varsa cuda:0, yoksa cpu; 'cpu' | 'cuda:0'
  target_classes    ['person']     yayınlanacak sınıf adları; [''] ya da boş = tüm sınıflar
  rate              5.0            Hz, çıkarım üst sınırı
  imgsz             640            çıkarım girdi boyutu
  cpu_threads       2              device=cpu iken torch iş parçacığı (OMP_NUM_THREADS üst sınır)
  image_topic       /camera/image_raw
  image_transport   raw            raw | compressed
  torso_lying_deg 45.0, torso_upright_deg 30.0, aspect_lying 1.5, kp_conf 0.5   (duruş eşikleri)
  alert_confidence  0.35           alarm için "yatan" tespitinin alt güveni
  alert_window      10             N: bakılan son çıkarım sayısı
  alert_min_hits    4              M: N'in içinde gereken "yatan" kare sayısı
  alert_period      2.0            s; alarm yineleme aralığı

Ortam: ultralytics + torch ~/fsb_venv içindedir ve düğüm PYTHONNOUSERSITE=1 ile başlatılmalıdır
(perception.launch.py bunu yapar); aksi halde ~/.local'daki pip OpenCV 5 cv_bridge'i bozar.
"""

from collections import deque
import json
import os
import threading
import time
from typing import Deque, Dict, List, Optional, Union

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from fsb_perception.posture import (  # noqa: I100 - yerel modül sonda
    classify, LYING, PostureParams, UPRIGHT)

COLORS = {LYING: (0, 0, 255), UPRIGHT: (0, 200, 0)}   # BGR; diğerleri gri
OTHER_COLOR = (160, 160, 160)
SKELETON = ((5, 6), (5, 11), (6, 12), (11, 12), (5, 7), (7, 9), (6, 8), (8, 10),
            (11, 13), (13, 15), (12, 14), (14, 16))
ImageMsg = Union[Image, CompressedImage]


class YoloNode(Node):

    def __init__(self) -> None:
        super().__init__('fsb_yolo_node')
        defaults = {
            'model_path': '~/yolov8n-pose.pt', 'confidence': 0.35, 'device': '',
            'target_classes': ['person'], 'rate': 5.0, 'imgsz': 640, 'cpu_threads': 2,
            'image_topic': '/camera/image_raw', 'image_transport': 'raw',
            'torso_lying_deg': 45.0, 'torso_upright_deg': 30.0, 'aspect_lying': 1.5,
            'kp_conf': 0.5, 'alert_confidence': 0.35, 'alert_window': 10,
            'alert_min_hits': 4, 'alert_period': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        gp = self.get_parameter
        self.confidence = float(gp('confidence').value)
        self.period = 1.0 / max(float(gp('rate').value), 0.1)
        self.imgsz = int(gp('imgsz').value)
        self.posture = PostureParams(float(gp('torso_lying_deg').value),
                                     float(gp('torso_upright_deg').value),
                                     float(gp('aspect_lying').value),
                                     float(gp('kp_conf').value))
        self.alert_conf = float(gp('alert_confidence').value)
        self.alert_min_hits = int(gp('alert_min_hits').value)
        self.alert_period = float(gp('alert_period').value)
        self._hits: Deque[bool] = deque(maxlen=max(1, int(gp('alert_window').value)))
        transport = str(gp('image_transport').value)
        if transport not in ('raw', 'compressed'):
            raise ValueError(f'image_transport={transport}: raw | compressed')

        if int(cv2.__version__.split('.')[0]) != 4:
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
        self.is_pose = self.model.task == 'pose'
        names: Dict[int, str] = dict(self.model.names)
        self.names = names
        wanted = [str(c) for c in gp('target_classes').value if str(c)]
        unknown = sorted(set(wanted) - set(names.values()))
        if unknown:
            raise ValueError(f'target_classes: modelde olmayan sınıf(lar): {unknown}')
        self.class_ids: Optional[List[int]] = [i for i, n in names.items() if n in wanted] or None

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest: Optional[ImageMsg] = None
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._last_alert = -1e9
        self.stats = {'frames': 0, 'infer_ms': 0.0, 'lying': 0}

        self.det_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.posture_pub = self.create_publisher(String, '/perception/posture', 10)
        self.img_pub = self.create_publisher(Image, '/perception/image_annotated', 1)
        self.cimg_pub = self.create_publisher(CompressedImage,
                                              '/perception/image_annotated/compressed', 1)
        self.alert_pub = self.create_publisher(Detection2D, '/security/alerts', 10)
        topic = str(gp('image_topic').value)
        if transport == 'compressed':
            topic += '/compressed'
            self.create_subscription(CompressedImage, topic, self._on_image,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

        # Isınma: ilk çıkarımın (CUDA başlatma, katman birleştirme) gecikmesi akışa yansımasın
        t0 = time.perf_counter()
        self._infer(np.zeros((self.imgsz, self.imgsz, 3), np.uint8))
        self.get_logger().info(
            f'YOLO hazır: {os.path.basename(model_path)} ({self.model.task}), device={device}, '
            f'girdi={topic}, conf≥{self.confidence}, {1.0 / self.period:.1f} Hz, '
            f'duruş: gövde≥{self.posture.torso_lying_deg:.0f}° | kutu oranı≥'
            f'{self.posture.aspect_lying}, alarm: yatan≥{self.alert_conf} × '
            f'{self.alert_min_hits}/{self._hits.maxlen} kare '
            f'(ısınma {time.perf_counter() - t0:.1f} s, cv2 {cv2.__version__})')
        if not self.is_pose:
            self.get_logger().warning('Model pose modeli değil: duruş yalnızca kutu oranından')
        self._worker = threading.Thread(target=self._run, name='yolo_infer', daemon=True)
        self._worker.start()
        self.create_timer(30.0, self._report)

    # ------------------------------------------------------------------ executor tarafı
    def _on_image(self, msg: ImageMsg) -> None:
        with self._lock:
            self._latest = msg
        self._new_frame.set()

    def _report(self) -> None:
        n, ms, ly = self.stats['frames'], self.stats['infer_ms'], self.stats['lying']
        self.stats = {'frames': 0, 'infer_ms': 0.0, 'lying': 0}
        if n:
            self.get_logger().info(f'son 30 s: {n} çıkarım ({n / 30.0:.1f} Hz), '
                                   f'ort. {ms / n:.1f} ms/kare, yatan görülen kare {ly}')

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

    def _decode(self, msg: ImageMsg) -> np.ndarray:
        if isinstance(msg, CompressedImage):
            bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f'sıkıştırılmış kare çözülemedi (format={msg.format})')
            return bgr
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _process(self, msg: ImageMsg) -> None:
        bgr = self._decode(msg)
        t0 = time.perf_counter()
        res = self._infer(bgr)
        self.stats['infer_ms'] += 1000.0 * (time.perf_counter() - t0)
        self.stats['frames'] += 1

        boxes = res.boxes
        xywh = boxes.xywh.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy().astype(int)
        kpts = res.keypoints.data.cpu().numpy() if self.is_pose and res.keypoints is not None \
            else None

        out = Detection2DArray()
        out.header = msg.header
        people: List[dict] = []
        best_lying: Optional[Detection2D] = None
        for k, ((cx, cy, w, h), conf, cls) in enumerate(zip(xywh, confs, clss)):
            name = self.names[int(cls)]
            kp = None if kpts is None or k >= len(kpts) else kpts[k]
            posture = None
            if name == 'person':
                posture, angle, aspect, cue = classify(float(w), float(h), kp, self.posture)
                name = {LYING: 'person_lying', UPRIGHT: 'person_upright'}.get(posture, 'person')
                people.append({
                    'bbox_cxcywh': [round(float(v), 1) for v in (cx, cy, w, h)],
                    'score': round(float(conf), 3), 'posture': posture, 'cue': cue,
                    'torso_angle_deg': None if angle is None else round(angle, 1),
                    'aspect': round(aspect, 2),
                    'keypoints': None if kp is None else np.round(kp, 2).tolist()})
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x, det.bbox.center.position.y = float(cx), float(cy)
            det.bbox.size_x, det.bbox.size_y = float(w), float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            out.detections.append(det)
            if posture == LYING and conf >= self.alert_conf and (
                    best_lying is None or conf > best_lying.results[0].hypothesis.score):
                best_lying = det
        self.det_pub.publish(out)
        stamp = msg.header.stamp
        self.posture_pub.publish(String(data=json.dumps(
            {'stamp': stamp.sec + stamp.nanosec * 1e-9, 'people': people})))

        # Zamansal doğrulama: son N çıkarımın M'sinde yatan insan
        self._hits.append(best_lying is not None)
        self.stats['lying'] += int(best_lying is not None)
        now = time.monotonic()
        if best_lying is not None and sum(self._hits) >= self.alert_min_hits and \
                now - self._last_alert >= self.alert_period:
            self._last_alert = now
            self.alert_pub.publish(best_lying)
            self.get_logger().warning(
                f'ALARM: yatan insan, güven {best_lying.results[0].hypothesis.score:.2f} '
                f'({sum(self._hits)}/{len(self._hits)} kare)', throttle_duration_sec=10.0)

        want_raw = self.img_pub.get_subscription_count() > 0
        want_jpg = self.cimg_pub.get_subscription_count() > 0
        if not (want_raw or want_jpg):
            return
        self._draw(bgr, out, kpts)
        if want_raw:
            ann = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            ann.header = msg.header
            self.img_pub.publish(ann)
        if want_jpg:
            ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                cm = CompressedImage()
                cm.header = msg.header
                cm.format = 'jpeg'
                cm.data = buf.tobytes()
                self.cimg_pub.publish(cm)

    def _draw(self, bgr: np.ndarray, out: Detection2DArray, kpts: Optional[np.ndarray]) -> None:
        for k, det in enumerate(out.detections):
            h = det.results[0].hypothesis
            posture = {'person_lying': LYING, 'person_upright': UPRIGHT}.get(h.class_id)
            color = COLORS.get(posture, OTHER_COLOR)
            x0 = int(det.bbox.center.position.x - det.bbox.size_x / 2)
            y0 = int(det.bbox.center.position.y - det.bbox.size_y / 2)
            x1, y1 = int(x0 + det.bbox.size_x), int(y0 + det.bbox.size_y)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), color, 2)
            cv2.putText(bgr, f'{h.class_id} {h.score:.2f}', (x0, max(y0 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            if kpts is None or k >= len(kpts):
                continue
            kp = kpts[k]
            for a, b in SKELETON:
                if kp[a][2] >= self.posture.kp_conf and kp[b][2] >= self.posture.kp_conf:
                    cv2.line(bgr, (int(kp[a][0]), int(kp[a][1])),
                             (int(kp[b][0]), int(kp[b][1])), color, 1, cv2.LINE_AA)

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
