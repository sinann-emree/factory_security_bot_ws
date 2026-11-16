"""
patrol_executor: patrol_planner'ın ürettiği waypoint yaml'ını Nav2 NavigateToPose ile gezer.

Modlar (parametre `mode`):
  idleness   (varsayılan) her adımda skor = ağırlık·idleness − yol_süresi en yüksek hedef seçilir;
             idleness = son ziyaretten beri geçen sim süresi,
             yol_süresi = Nav2 yol uzunluğu / nominal hız.
  sequential yaml sırası (planlayıcının TSP turu), robota en yakın noktadan başlayarak döngü.
  random     her turda rastgele permütasyon (seed parametresi).
Bir "tur" = tüm hedeflerin (waypoint + öncelikli nokta) son tur başından beri en az bir kez ziyaret
edilmesi ya da atlanması. Ulaşılamayan / zaman aşımına uğrayan hedef ATLANIR ve loglanır, tur iptal
olmaz. Öncelikli noktalar dünyaya özel ayrı bir yaml'dan gelir (priority_file), ağırlıkları
idleness'ı çarpar; diğer hedeflere uzaklıkları açılışta Nav2 ComputePathToPose ile ölçülür.

Güvenlik alarmı (/security/alerts, vision_msgs/Detection2D; fsb_perception yolo_node yayınlar):
  1. DUR    — etkin Nav2 hedefi iptal edilir (controller sıfır hız basar), hedef hatırlanır.
  2. UYAR   — WARN logu + RViz'de robotun konumunda kırmızı "ALARM" işareti (patrol/waypoints).
  3. KAYDET — alert_log_dir/events.jsonl'a bir satır (zaman, sınıf, güven, kutu, robotun map pozu,
              tur, hedef, iptal gecikmesi) + alarm anındaki işaretli kamera karesi (.jpg).
  4. DEVAM  — alarmdan alert_hold sim-s sonra (ve robot durmuşsa) aynı hedef yeniden gönderilir.
  Olay bittikten sonra alert_cooldown sim-s boyunca yeni alarmlar yok sayılır (aynı kişi için
  tekrar tekrar durmasın); yok sayılanlar sayılır. Olay sırasında hedef zaman aşımı işlemez.

Parametreler: waypoints_file, priority_file (''), mode, loops (0 = sonsuz), goal_timeout (sim-s),
  nominal_speed (m/s), seed, frame_id (map), base_frame (base_footprint),
  react_to_alerts (true), alert_topic (/security/alerts), alert_hold (5.0 sim-s),
  alert_cooldown (120.0 sim-s), alert_max_age (2.0 sim-s; daha eski kareye ait alarm yok sayılır),
  alert_log_dir (~/.ros/fsb_security),
  alert_image_topic (/perception/image_annotated; '' = kare kaydetme).
"""

import json
import math
import os
import random
import time
from typing import Any, Dict, List, Optional, Set

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import numpy as np
from PIL import Image as PilImage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2D
from visualization_msgs.msg import Marker, MarkerArray
import yaml

# RViz renkleri (r, g, b): sıradaki hedef / bu turda ziyaret / atlandı / bekliyor / öncelikli
COLORS = {'goal': (1.0, 0.55, 0.0), 'visited': (0.15, 0.75, 0.25), 'skipped': (0.9, 0.1, 0.1),
          'pending': (0.15, 0.45, 0.95), 'priority': (0.6, 0.2, 0.8)}


class Target:
    def __init__(self, name: str, x: float, y: float, yaw: float, weight: float = 1.0,
                 priority: bool = False) -> None:
        self.name, self.x, self.y, self.yaw = name, x, y, yaw
        self.weight, self.priority = weight, priority
        self.last_visit = 0.0
        self.visits = 0
        self.skips = 0


class PatrolExecutor(Node):

    def __init__(self) -> None:
        super().__init__('fsb_patrol_executor')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('priority_file', '')
        self.declare_parameter('mode', 'idleness')
        self.declare_parameter('loops', 0)
        self.declare_parameter('goal_timeout', 180.0)
        self.declare_parameter('nominal_speed', 0.2)
        self.declare_parameter('seed', 0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('react_to_alerts', True)
        self.declare_parameter('alert_topic', '/security/alerts')
        self.declare_parameter('alert_hold', 5.0)
        self.declare_parameter('alert_cooldown', 120.0)
        self.declare_parameter('alert_max_age', 2.0)
        self.declare_parameter('alert_log_dir', '~/.ros/fsb_security')
        self.declare_parameter('alert_image_topic', '/perception/image_annotated')
        gp = self.get_parameter
        self.mode: str = gp('mode').value
        if self.mode not in ('idleness', 'sequential', 'random'):
            raise ValueError(f'mode={self.mode}: idleness | sequential | random')
        self.loops = int(gp('loops').value)
        self.goal_timeout = float(gp('goal_timeout').value)
        self.v_nom = float(gp('nominal_speed').value)
        self.frame_id: str = gp('frame_id').value
        self.base_frame: str = gp('base_frame').value
        self.rng = random.Random(int(gp('seed').value))

        wf = gp('waypoints_file').value
        if not wf:
            raise ValueError('waypoints_file parametresi gerekli')
        with open(wf, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        self.targets: List[Target] = [Target(w['name'], float(w['x']), float(w['y']),
                                             float(w['yaw'])) for w in doc['waypoints']]
        n = len(self.targets)
        self.dist = np.full((n, n), np.nan)
        if 'path_lengths_m' in doc:
            self.dist[:n, :n] = np.array(doc['path_lengths_m'], dtype=float)
        self.get_logger().info(
            f'{n} waypoint ({wf}, maliyet={doc.get("cost_source", "?")}, '
            f'planlanan kapsama={doc.get("summary", {}).get("coverage_total", "?")})')
        pf = gp('priority_file').value
        if pf:
            with open(pf, 'r', encoding='utf-8') as f:
                pdoc = yaml.safe_load(f) or {}
            for i, pp in enumerate(pdoc.get('priority_points', [])):
                self.targets.append(Target(pp.get('name', f'P{i + 1}'), float(pp['x']),
                                           float(pp['y']), float(pp.get('yaw', 0.0)),
                                           float(pp.get('weight', 2.0)), True))
            self.get_logger().info(f'{len(self.targets) - n} öncelikli nokta ({pf})')

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lap = 0
        self.lap_done: Set[int] = set()
        self.lap_visited: List[str] = []
        self.lap_skipped: List[str] = []
        self.lap_start_sim = 0.0
        self.lap_start_wall = 0.0
        self.lap_max_idle = 0.0
        self.order: List[int] = []
        self.current: Optional[int] = None   # robotun bulunduğu hedef
        self.goal_idx: Optional[int] = None
        self.goal_start = 0.0
        self.goal_handle = None
        self.cooldown: Dict[int, int] = {}   # atlanan hedef → kalan bekleme adımı
        self.finished = False
        self.total = {'visits': 0, 'skips': 0, 'laps_ok': 0, 'alerts': 0, 'alerts_ignored': 0}
        self.create_timer(1.0, self._watchdog)
        # Güvenlik alarmı durumu
        self.alert_hold = float(gp('alert_hold').value)
        self.alert_cooldown = float(gp('alert_cooldown').value)
        self.alert_max_age = float(gp('alert_max_age').value)
        self.alert_log_dir = os.path.expanduser(str(gp('alert_log_dir').value))
        self.alert_image_topic = str(gp('alert_image_topic').value)
        self.incident: Optional[Dict[str, Any]] = None   # süren olay
        self.incident_end = -math.inf                    # son olayın bittiği sim zamanı
        self.deferred_next = False                       # olay sırasında ertelenen _next()
        self.alert_marks: List[tuple] = []               # (x, y, metin) — RViz
        self._img_sub = None
        if bool(gp('react_to_alerts').value):
            self.create_subscription(Detection2D, str(gp('alert_topic').value),
                                     self._on_alert, 10)
            self.create_timer(0.2, self._incident_tick)
        # RViz: waypoint'ler (yön oku + etiket), durum renkli.
        # Transient-local: geç açılan RViz de son durumu alır.
        self.marker_pub = self.create_publisher(
            MarkerArray, 'patrol/waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._publish_markers()

    # ------------------------------------------------------------------ yardımcılar
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, t: Target) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = t.x, t.y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(t.yaw / 2), math.cos(t.yaw / 2)
        return ps

    def _robot_pose(self, timeout: float = 0.5) -> Optional[tuple]:
        """(x, y, yaw) map çerçevesinde; TF yoksa None."""
        try:
            tr = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time(),
                                                 timeout=Duration(seconds=timeout))
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tr.transform.translation.x, tr.transform.translation.y, yaw
        except Exception:  # noqa: BLE001 - TF henüz yok
            return None

    def _robot_xy(self) -> Optional[tuple]:
        pose = self._robot_pose()
        return None if pose is None else pose[:2]

    def _path_length(self, a: Target, b: Target) -> float:
        g = ComputePathToPose.Goal()
        g.start, g.goal, g.use_start = self._pose(a), self._pose(b), True
        fut = self.planner.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return float('nan')
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10.0)
        r = rf.result()
        if r is None or r.status != GoalStatus.STATUS_SUCCEEDED or len(r.result.path.poses) < 2:
            return float('nan')
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in r.result.path.poses])
        return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())

    def _complete_distances(self) -> None:
        """Öncelikli noktalar (ve eksik çiftler) için Nav2 yol uzunlukları; olmazsa öklid × 1.3."""
        n = len(self.targets)
        d = np.full((n, n), np.nan)
        m = self.dist.shape[0]
        d[:m, :m] = self.dist
        missing = [(i, j) for i in range(n) for j in range(n) if i != j and np.isnan(d[i, j])]
        if missing:
            ok = self.planner.wait_for_server(timeout_sec=10.0)
            for i, j in missing:
                v = self._path_length(self.targets[i], self.targets[j]) if ok else float('nan')
                if np.isnan(v):
                    v = 1.3 * math.hypot(self.targets[i].x - self.targets[j].x,
                                         self.targets[i].y - self.targets[j].y)
                d[i, j] = v
            self.get_logger().info(f'{len(missing)} eksik yol uzunluğu Nav2 ile tamamlandı')
        np.fill_diagonal(d, 0.0)
        self.dist = d

    def _publish_markers(self) -> None:
        arr = MarkerArray()
        skipped = set(self.lap_skipped)
        visited = set(self.lap_visited)
        for k, tg in enumerate(self.targets):
            if k == self.goal_idx:
                state = 'goal'
            elif tg.name in skipped:
                state = 'skipped'
            elif tg.name in visited:
                state = 'visited'
            else:
                state = 'priority' if tg.priority else 'pending'
            r, g, b = COLORS[state]
            arrow = Marker()
            arrow.header.frame_id = self.frame_id  # stamp 0 → RViz en güncel TF'i kullanır
            arrow.ns, arrow.id, arrow.type, arrow.action = 'waypoint', k, Marker.ARROW, Marker.ADD
            arrow.pose = self._pose(tg).pose
            big = state == 'goal'
            arrow.scale.x, arrow.scale.y, arrow.scale.z = (0.6, 0.12, 0.12) if big else \
                (0.4, 0.07, 0.07)
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.0
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns, text.id, text.type, text.action = 'label', k, Marker.TEXT_VIEW_FACING, \
                Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = tg.x, tg.y, 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35 if big else 0.25
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
            text.text = tg.name
            arr.markers += [arrow, text]
        for k, (x, y, label) in enumerate(self.alert_marks[-20:]):
            for ns, mtype, z, size in (('alert', Marker.SPHERE, 0.15, 0.3),
                                       ('alert_label', Marker.TEXT_VIEW_FACING, 0.8, 0.3)):
                mk = Marker()
                mk.header.frame_id = self.frame_id
                mk.ns, mk.id, mk.type, mk.action = ns, k, mtype, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = size
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.0, 0.0, 0.9
                mk.text = label
                arr.markers.append(mk)
        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------ akış
    def start(self) -> None:
        while not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('navigate_to_pose action sunucusu bekleniyor...')
        self._complete_distances()
        xy = None
        t0 = time.monotonic()
        while xy is None and time.monotonic() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.2)
            xy = self._robot_xy()
        if xy is None:
            self.get_logger().warning('map→base_footprint yok; başlangıç W01 kabul edildi')
            self.start_idx = 0
        else:
            self.start_idx = int(np.argmin([math.hypot(t.x - xy[0], t.y - xy[1])
                                            for t in self.targets]))
        t = self.now()
        for tg in self.targets:
            tg.last_visit = t
        self.get_logger().info(f'Mod: {self.mode}, tur: {self.loops or "sonsuz"}, '
                               f'başlangıca en yakın hedef: {self.targets[self.start_idx].name}')
        self._new_lap()
        self._next()

    def _new_lap(self) -> None:
        self.lap += 1
        self.lap_done = set()
        self.lap_visited, self.lap_skipped = [], []
        self.lap_start_sim, self.lap_start_wall = self.now(), time.monotonic()
        self.lap_max_idle = 0.0
        self._publish_markers()
        n = len(self.targets)
        if self.mode == 'sequential':
            s = self.start_idx if self.current is None else (self.current + 1) % n
            self.order = [(s + k) % n for k in range(n)]
        elif self.mode == 'random':
            self.order = list(range(n))
            self.rng.shuffle(self.order)
        self.get_logger().info(f'=== Tur {self.lap} başladı ===')

    def _choose(self) -> Optional[int]:
        n = len(self.targets)
        if self.mode in ('sequential', 'random'):
            rest = [i for i in self.order if i not in self.lap_done]
            return rest[0] if rest else None
        t = self.now()
        best, best_s = None, -math.inf
        for i in range(n):
            if i == self.current or self.cooldown.get(i, 0) > 0:
                continue
            tg = self.targets[i]
            d = (self.dist[self.current, i] if self.current is not None
                 else self.dist[self.start_idx, i] if i != self.start_idx else 0.0)
            s = tg.weight * (t - tg.last_visit) - d / self.v_nom
            if s > best_s:
                best, best_s = i, s
        if best is None:  # hepsi beklemede: beklemeyi sıfırla
            self.cooldown.clear()
            return self._choose()
        return best

    def _next(self) -> None:
        if self.incident is not None:   # olay bitince _resume() çağırır
            self.deferred_next = True
            return
        if len(self.lap_done) == len(self.targets):
            self._finish_lap()
            if self.finished:
                return
        i = self._choose()
        if i is None:
            self._finish_lap()
            if self.finished:
                return
            i = self._choose()
        for k in list(self.cooldown):
            self.cooldown[k] -= 1
            if self.cooldown[k] <= 0:
                del self.cooldown[k]
        tg = self.targets[i]
        idle = self.now() - tg.last_visit
        self.lap_max_idle = max(self.lap_max_idle, idle)
        self.get_logger().info(f'→ {tg.name} ({tg.x:+.2f}, {tg.y:+.2f}) idleness={idle:.0f} s'
                               + (' [öncelikli]' if tg.priority else ''))
        self._send_goal(i)

    def _send_goal(self, i: int) -> None:
        self.goal_idx, self.goal_start = i, self.now()
        g = NavigateToPose.Goal()
        g.pose = self._pose(self.targets[i])
        self._publish_markers()
        self.nav.send_goal_async(g).add_done_callback(self._on_accept)

    def _on_accept(self, fut) -> None:
        gh = fut.result()
        if not gh.accepted:
            self._skip('hedef reddedildi')
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)
        if self.incident is not None:   # alarm, hedef kabul edilmeden hemen önce geldi
            gh.cancel_goal_async()

    def _on_result(self, fut) -> None:
        res = fut.result()
        self.goal_handle = None
        if self.goal_idx is None:
            return
        if self.incident is not None and res.status == GoalStatus.STATUS_CANCELED:
            # Alarm için bilerek iptal edildi: hedef atlanmaz, _resume() yeniden gönderir
            self.incident.setdefault('stopped_sim', self.now())
            return
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            i = self.goal_idx
            tg = self.targets[i]
            tg.last_visit = self.now()
            tg.visits += 1
            self.total['visits'] += 1
            self.current = i
            self.lap_done.add(i)
            self.lap_visited.append(tg.name)
            self.get_logger().info(
                f'✓ {tg.name} ulaşıldı ({self.now() - self.goal_start:.1f} sim-s)')
            self.goal_idx = None
            self._publish_markers()
            self._next()
        elif res.status == GoalStatus.STATUS_CANCELED and getattr(self, '_timed_out', False):
            self._timed_out = False
            self._skip(f'zaman aşımı ({self.goal_timeout:.0f} sim-s)')
        else:
            code = getattr(res.result, 'error_code', '?')
            msg = getattr(res.result, 'error_msg', '')
            self._skip(f'Nav2 durum={res.status} hata={code} {msg}'.strip())

    def _skip(self, why: str) -> None:
        i = self.goal_idx
        self.goal_idx = None
        tg = self.targets[i]
        tg.skips += 1
        self.total['skips'] += 1
        self.lap_done.add(i)
        self.lap_skipped.append(tg.name)
        self.cooldown[i] = 3
        self.get_logger().warning(f'✗ ATLANDI: {tg.name} ({tg.x:+.2f}, {tg.y:+.2f}) — {why}; '
                                  'tur devam ediyor')
        self._publish_markers()
        self._next()

    # ------------------------------------------------------------------ güvenlik alarmı
    def _on_alert(self, msg: Detection2D) -> None:
        if self.lap == 0 or self.finished or not msg.results:
            return
        now = self.now()
        if self.incident is not None or now - self.incident_end < self.alert_cooldown:
            self.total['alerts_ignored'] += 1
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now - stamp > self.alert_max_age:
            # Kuyrukta beklemiş eski alarm (ör. açılışta Nav2 beklenirken birikenler)
            self.total['alerts_ignored'] += 1
            return
        hyp = msg.results[0].hypothesis
        pose = self._robot_pose(timeout=0.0)
        self.total['alerts'] += 1
        goal = self.targets[self.goal_idx].name if self.goal_idx is not None else None
        self.incident = {
            'id': self.total['alerts'], 'wall_time': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'sim_time': round(now, 3), 'class': hyp.class_id, 'score': round(hyp.score, 4),
            'bbox_cxcywh': [round(v, 1) for v in (msg.bbox.center.position.x,
                                                  msg.bbox.center.position.y,
                                                  msg.bbox.size_x, msg.bbox.size_y)],
            'detection_stamp': round(stamp, 3),
            'robot_map_pose': None if pose is None else [round(v, 3) for v in pose],
            'lap': self.lap, 'goal': goal, 'image': None,
        }
        # 1) DUR
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        elif self.goal_idx is None:
            self.incident['stopped_sim'] = now   # zaten hedefsiz
        # 2) UYAR
        where = 'konum bilinmiyor' if pose is None else f'robot ({pose[0]:+.2f}, {pose[1]:+.2f})'
        self.get_logger().warning(
            f'!!! GÜVENLİK ALARMI #{self.incident["id"]}: {hyp.class_id} güven {hyp.score:.2f} — '
            f'{where}, hedef {goal}: DEVRİYE DURDURULDU ({self.alert_hold:.0f} sim-s)')
        if pose is not None:
            self.alert_marks.append((pose[0], pose[1],
                                     f'ALARM #{self.incident["id"]} {hyp.class_id} '
                                     f'{hyp.score:.2f}'))
            self._publish_markers()
        # 3) KAYDET (kare): alarm sonrası ilk işaretli kare
        if self.alert_image_topic and self._img_sub is None:
            self._img_sub = self.create_subscription(Image, self.alert_image_topic,
                                                     self._on_alert_image, 1)

    def _on_alert_image(self, msg: Image) -> None:
        inc = self.incident
        if inc is None or inc['image'] is not None or msg.encoding not in ('bgr8', 'rgb8'):
            return
        try:
            os.makedirs(self.alert_log_dir, exist_ok=True)
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            a = a.reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                a = a[..., ::-1]
            path = os.path.join(self.alert_log_dir,
                                f'alarm_{time.strftime("%Y%m%d_%H%M%S")}_{inc["id"]:03d}.jpg')
            PilImage.fromarray(np.ascontiguousarray(a)).save(path, quality=90)
            inc['image'] = path
        except Exception as e:  # noqa: BLE001 - kayıt hatası devriyeyi düşürmesin
            self.get_logger().error(f'alarm karesi kaydedilemedi: {e!r}')
            inc['image'] = ''

    def _incident_tick(self) -> None:
        inc = self.incident
        if inc is None:
            return
        now = self.now()
        if now - inc['sim_time'] < self.alert_hold or self.goal_handle is not None:
            return   # bekleme sürüyor ya da iptal sonucu henüz gelmedi
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        stopped = inc.pop('stopped_sim', None)
        # Nav2'nin iptali onaylama süresi (fiziksel durma ayrıca yavaşlama süresi kadar sürer)
        inc['cancel_latency_s'] = None if stopped is None else round(stopped - inc['sim_time'], 3)
        inc['resumed_sim_time'] = round(now, 3)
        inc['image'] = inc['image'] or None
        try:
            os.makedirs(self.alert_log_dir, exist_ok=True)
            with open(os.path.join(self.alert_log_dir, 'events.jsonl'), 'a',
                      encoding='utf-8') as f:
                f.write(json.dumps(inc, ensure_ascii=False) + '\n')
        except OSError as e:
            self.get_logger().error(f'alarm kaydı yazılamadı: {e!r}')
        self.incident, self.incident_end = None, now
        self._resume(inc)

    def _resume(self, inc: Dict[str, Any]) -> None:
        """4) DEVAM: yarım kalan hedef yeniden gönderilir (ya da ertelenen seçim yapılır)."""
        deferred, self.deferred_next = self.deferred_next, False
        tail = f'kayıt: {self.alert_log_dir}/events.jsonl, kare: {inc["image"]}'
        if self.goal_idx is not None and not deferred:
            self.get_logger().info(f'Alarm #{inc["id"]} kaydedildi; devriye sürüyor → '
                                   f'{self.targets[self.goal_idx].name} ({tail})')
            self._send_goal(self.goal_idx)
        else:
            self.get_logger().info(f'Alarm #{inc["id"]} kaydedildi; devriye sürüyor ({tail})')
            self._next()

    def _watchdog(self) -> None:
        if self.incident is not None:
            return
        if self.goal_idx is not None and self.goal_handle is not None and \
                self.now() - self.goal_start > self.goal_timeout:
            self._timed_out = True
            self.goal_handle.cancel_goal_async()

    def _finish_lap(self) -> None:
        n = len(self.targets)
        ok = len(self.lap_skipped) == 0
        self.total['laps_ok'] += int(ok)
        self.get_logger().info(
            f'Tur {self.lap} TAMAM (sim {self.now() - self.lap_start_sim:.1f} s / duvar '
            f'{time.monotonic() - self.lap_start_wall:.1f} s): '
            f'ziyaret {len(set(self.lap_visited))}/{n}, '
            f'atlanan {self.lap_skipped or "yok"}, maks idleness {self.lap_max_idle:.0f} s')
        if self.loops and self.lap >= self.loops:
            self.finished = True
            self.get_logger().info(
                f'DEVRİYE BİTTİ: {self.lap} tur, eksiksiz tur {self.total["laps_ok"]}/{self.lap}, '
                f'toplam ziyaret {self.total["visits"]}, atlama {self.total["skips"]}, '
                f'alarm {self.total["alerts"]} (yok sayılan {self.total["alerts_ignored"]})')
            return
        self._new_lap()

    def cancel(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PatrolExecutor()
    try:
        node.start()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancel()
    except Exception:
        if rclpy.ok():   # kapanış sırasında (SIGINT) geçersiz bağlam hatası yutulur
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
