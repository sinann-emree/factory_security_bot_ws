"""
Yatan insan olayı yönetimi (patrol_executor'a karışan sınıf — IncidentMixin).

Akış:
  /security/alerts (class_id = person_lying) → kişinin map konumu (LiDAR + kamera yönü)
    ├─ konum, etkin bir kaydın birleştirme yarıçapındaysa: yalnızca "son görülme" güncellenir
    │  (yeni bildirim / durma YOK — liste konumları devriyede tekrar alarm üretmez)
    └─ yeni kişi: listeye eklenir, BİR KEZ bildirilir, devriye hedefi iptal edilir ve
       KAYIT GÖREVİ başlar: kişinin çevresinde inspect_radius uzaklıkta inspect_views farklı
       açıdan durulur; her açıda observe_time boyunca tespitler sayılır, işaretli kare ve duruş
       (pose) kaydı alınır. Sonra devriye kaldığı hedeften sürer.
  revisit_interval sonra (iki devriye hedefi arasında) REVISIT GÖREVİ: konumu görebilen bir bakış
  noktasına gidilir ve gözlenir:
    - kişi hâlâ yerde  → doğrulama sayısı +1, bildirim yenilenir, yeni revisit zamanlanır
    - kişi görülmedi   → SADECE konum kamera görüş alanındaysa, LiDAR'da önünde engel yoksa ve
                         konum boşsa kayıt "çözüldü" olur. Aksi halde (görüş doğrulanamadı)
                         kayıt listede KALIR; sıradaki bakış noktası denenir, hiçbiri olmazsa
                         revisit_retry sonra yeniden denenir.

Görev durumu self.mission sözlüğünde tutulur; phase: cancelling → nav → observe → (sonraki açı).
"""

import json
import math
import os
import time
from typing import Any, Dict, List, Optional

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image as PilImage
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker

from fsb_patrol import incident_geometry as geo  # noqa: I100 - yerel modüller sonda
from fsb_patrol.watchlist import WatchEntry, Watchlist

INCIDENT_PARAMS = {
    'react_to_alerts': True,
    'alert_topic': '/security/alerts',
    'alert_max_age': 2.0,            # sim-s; daha eski kareye ait alarm yok sayılır
    'alert_log_dir': '~/.ros/fsb_security',
    'alert_image_topic': '/perception/image_annotated',
    'detections_topic': '/perception/detections',
    'posture_topic': '/perception/posture',
    'scan_topic': '/scan',
    'camera_info_topic': '/camera/camera_info',
    'costmap_topic': '/global_costmap/costmap',
    'camera_frame': 'camera_optical_link',
    'lidar_frame': 'lidar_link',
    'lying_class': 'person_lying',
    'merge_radius': 1.0,             # m; bu yarıçaptaki tespitler aynı kişidir
    'inspect_radius': 2.0,           # m; kayıt/revisit bakış noktalarının kişiye uzaklığı
    'inspect_views': 3,              # kayıt için farklı açı sayısı
    'observe_time': 4.0,             # sim-s; her bakış noktasında gözlem süresi
    'observe_settle': 1.0,           # sim-s; durduktan sonra sayılmayan süre
    'observe_min_hits': 3,           # "görüldü" için konuma düşen yatan-insan tespiti sayısı
    'revisit_interval': 300.0,       # sim-s; kayıttan / doğrulamadan sonra revisit
    'revisit_retry': 60.0,           # sim-s; görüş doğrulanamayınca yeniden deneme
    'add_max_range': 6.5,            # m; bundan uzaktan konumlanan tespit yeni kayıt açmaz
    'unconfirmed_merge_radius': 2.0,  # m; kayıtta doğrulanamamış kayda bu kadar yakın yeni tespit
    'keepout_radius': 1.0,           # m; etkin kaydın çevresinde costmap yasak bölgesi (0: kapalı)
    'keepout_topic': '/security/keepout',
    'view_max_range': 4.0,           # m; "kişi yok" kararı için en uzak bakış mesafesi
}


class IncidentMixin:
    """PatrolExecutor'a yatan-insan olay akışını ekler (Node ile birlikte kullanılır)."""

    # ------------------------------------------------------------------ kurulum
    def _init_incidents(self) -> None:
        for name, default in INCIDENT_PARAMS.items():
            self.declare_parameter(name, default)
        gp = self.get_parameter
        self.ip: Dict[str, Any] = {k: gp(k).value for k in INCIDENT_PARAMS}
        self.watch = Watchlist(str(self.ip['alert_log_dir']), float(self.ip['merge_radius']))
        self.mission: Optional[Dict[str, Any]] = None
        self._scan: Optional[LaserScan] = None
        self._cam: Optional[CameraInfo] = None
        self._grid: Optional[geo.Grid] = None
        self._static: Dict[str, tuple] = {}
        self._img_sub = None
        self._last_image: Optional[Image] = None
        self._last_posture: Optional[dict] = None
        self.total.update({'new_persons': 0, 'alerts_known': 0, 'alerts_unlocated': 0,
                           'alerts_deferred': 0, 'alerts_far': 0, 'relocated': 0,
                           'revisits': 0, 'confirmed': 0,
                           'resolved': 0, 'unverified': 0})
        if not bool(self.ip['react_to_alerts']):
            return
        self.create_subscription(Detection2D, str(self.ip['alert_topic']), self._on_alert, 10)
        self.create_subscription(Detection2DArray, str(self.ip['detections_topic']),
                                 self._on_detections, 10)
        self.create_subscription(String, str(self.ip['posture_topic']), self._on_posture, 10)
        self.create_subscription(LaserScan, str(self.ip['scan_topic']),
                                 lambda m: setattr(self, '_scan', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.ip['camera_info_topic']),
                                 lambda m: setattr(self, '_cam', m), qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, str(self.ip['costmap_topic']), self._on_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.2, self._mission_tick)
        self._keepout_pub = self.create_publisher(PointCloud2, str(self.ip['keepout_topic']), 1)
        self._keepout_used = False
        if float(self.ip['keepout_radius']) > 0.0:
            self.create_timer(1.0, self._publish_keepout)

    def _load_watchlist(self) -> None:
        """Devriye başlarken (saat geçerliyken) önceki oturumun listesini yükler."""
        n = self.watch.load(self.now(), float(self.ip['revisit_interval']))
        if n:
            self.get_logger().info(f'Olay listesi yüklendi: {n} etkin kayıt ({self.watch.path})')
            self._publish_markers()

    def _on_costmap(self, m: OccupancyGrid) -> None:
        data = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        self._grid = geo.Grid(data, m.info.resolution, m.info.origin.position.x,
                              m.info.origin.position.y)

    # ------------------------------------------------------------------ geometri yardımcıları
    def _static_pose(self, frame: str, axis: tuple) -> Optional[tuple]:
        """Sabit çerçevenin base'deki (x, y, yaw, z)'si; yaw = verilen eksenin yatay yönü."""
        if frame in self._static:
            return self._static[frame]
        try:
            tr = self.tf_buffer.lookup_transform(self.base_frame, frame, Time(),
                                                 timeout=Duration(seconds=0.0))
        except Exception:  # noqa: BLE001 - TF henüz yok
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        x, y, z = axis
        # v' = q v q* (yalnızca x, y bileşenleri gerekli)
        vx = (1 - 2 * (q.y * q.y + q.z * q.z)) * x + 2 * (q.x * q.y - q.w * q.z) * y \
            + 2 * (q.x * q.z + q.w * q.y) * z
        vy = 2 * (q.x * q.y + q.w * q.z) * x + (1 - 2 * (q.x * q.x + q.z * q.z)) * y \
            + 2 * (q.y * q.z - q.w * q.x) * z
        self._static[frame] = (t.x, t.y, math.atan2(vy, vx), t.z)
        return self._static[frame]

    def _sensor_frames(self) -> Optional[tuple]:
        cam = self._static_pose(str(self.ip['camera_frame']), (0.0, 0.0, 1.0))
        lid = self._static_pose(str(self.ip['lidar_frame']), (1.0, 0.0, 0.0))
        return None if cam is None or lid is None else (cam, lid)

    def _scan_pts(self, lidar: tuple, keep_far: bool = False) -> np.ndarray:
        s = self._scan
        if s is None:
            return np.zeros((0, 2))
        return geo.scan_points(s.ranges, s.angle_min, s.angle_increment, lidar[:3],
                               s.range_min, s.range_max, keep_far=keep_far)

    def _locate(self, det: Detection2D) -> Optional[tuple]:
        """Tespitin map konumu: (x, y, yöntem, nokta sayısı, gövde noktaları) ya da None."""
        frames, cam, robot = self._sensor_frames(), self._cam, self._robot_pose(timeout=0.0)
        if frames is None or cam is None or robot is None:
            return None
        cam_b, lid_b = frames
        fx, fy, cx, cy = cam.k[0], cam.k[4], cam.k[2], cam.k[5]
        u, w = det.bbox.center.position.x, det.bbox.size_x
        left = geo.pixel_bearing(u - 0.4 * w, fx, cx, cam_b[2])
        right = geo.pixel_bearing(u + 0.4 * w, fx, cx, cam_b[2])
        hit = geo.locate_from_scan(self._scan_pts(lid_b), cam_b[:3], left, right)
        method, n, cluster = 'lidar', 0, []
        if hit is not None:
            bx, by, n, cl = hit
            cluster = [geo.to_frame(robot, float(p[0]), float(p[1])) for p in cl]
        else:
            g = geo.ground_ray(u, det.bbox.center.position.y + det.bbox.size_y / 2.0,
                               fx, fy, cx, cy, cam_b[:3], cam_b[3])
            if g is None:
                return None
            bx, by = g
            method = 'ground_ray'
        mx, my = geo.to_frame(robot, bx, by)
        return mx, my, method, n, cluster

    # ------------------------------------------------------------------ algı girişleri
    def _fresh(self, stamp) -> bool:
        return self.now() - (stamp.sec + stamp.nanosec * 1e-9) <= float(self.ip['alert_max_age'])

    def _on_alert(self, msg: Detection2D) -> None:
        if self.lap == 0 or self.finished or not msg.results:
            return
        hyp = msg.results[0].hypothesis
        if hyp.class_id != self.ip['lying_class'] or not self._fresh(msg.header.stamp):
            return
        loc = self._locate(msg)
        if loc is None:
            self.total['alerts_unlocated'] += 1
            self.get_logger().warning('Yatan insan alarmı: konum hesaplanamadı (TF / tarama / '
                                      'kamera bilgisi yok); yok sayıldı',
                                      throttle_duration_sec=10.0)
            return
        x, y, method, n, cluster = loc
        known = self.watch.find(x, y)
        if known is None and method != 'lidar':
            # Zemin ışını menzili güvenilmez (kamera yere çok yakın): yalnızca yönü kullanılır.
            # Bakış doğrultusu üzerinde bilinen bir kişi varsa aynı kişidir.
            rp = self._robot_pose(timeout=0.0)
            near = [q for q in self.watch.active() if rp is not None and
                    self.watch.ray_distance(q, rp[0], rp[1], x, y) <= self.watch.merge_radius]
            known = near[0] if near else None
        if known is not None:
            known.last_seen_sim = round(self.now(), 3)
            self.total['alerts_known'] += 1
            return
        if self.mission is not None:
            self.total['alerts_deferred'] += 1   # görev bitince yeniden görülürse eklenir
            return
        pose = self._robot_pose(timeout=0.0)
        rng = math.inf if pose is None else math.hypot(x - pose[0], y - pose[1])
        if rng > float(self.ip['add_max_range']):
            # Uzak tespitte kutu–LiDAR eşlemesi güvenilmez (ölçüm: 10 m'den 2.7 m hatalı konum).
            # Kayıt açılmaz; devriye yaklaşınca yeniden görülür.
            self.total['alerts_far'] += 1
            return
        stale = self._unconfirmed_near(x, y)
        if stale is not None:
            # Kayıtta hiçbir açıdan doğrulanamamış kayıt büyük olasılıkla yanlış konumlandı:
            # yeni kayıt açmak yerine o kayıt yeni konuma taşınır ve kayıt yinelenir.
            old = (stale.x, stale.y)
            stale.x, stale.y, stale.extent = round(x, 3), round(y, 3), []
            self.watch.set_extent(stale, cluster)
            stale.last_seen_sim = round(self.now(), 3)
            self.total['relocated'] += 1
            self.get_logger().info(
                f'#{stale.id}: doğrulanamamış kayıt ({old[0]:+.2f}, {old[1]:+.2f}) → '
                f'({stale.x:+.2f}, {stale.y:+.2f}) konumuna taşındı; kayıt yineleniyor')
            self.watch.log_event('relocated', id=stale.id, x=stale.x, y=stale.y,
                                 old=list(old), sim_time=round(self.now(), 3), method=method)
            self.watch.save()
            self._publish_markers()
            self._start_mission('inspect', stale)
            return
        e = self.watch.add(x, y, self.now())
        self.watch.set_extent(e, cluster)
        self.total['new_persons'] += 1
        self.get_logger().warning(
            f'!!! YATAN İNSAN #{e.id}: güven {hyp.score:.2f}, map konumu ({e.x:+.2f}, {e.y:+.2f}) '
            f'[{method}, {n} nokta] — listeye eklendi, devriye durduruldu, kayıt alınıyor')
        self.watch.log_event('added', id=e.id, x=e.x, y=e.y, sim_time=round(self.now(), 3),
                             score=round(hyp.score, 3), method=method,
                             robot_map_pose=None if pose is None else [round(v, 3) for v in pose],
                             lap=self.lap)
        self.watch.save()
        self._publish_markers()
        self._start_mission('inspect', e)

    def _unconfirmed_near(self, x: float, y: float) -> Optional[WatchEntry]:
        """Kaydı alınmış ama hiç doğrulanmamış, (x, y)'ye yakın etkin kayıt (yoksa None)."""
        r = float(self.ip['unconfirmed_merge_radius'])
        for e in self.watch.active():
            if e.observations and not any(o.get('seen') for o in e.observations) and \
                    e.confirmations == 0 and self.watch.distance(e, x, y) <= r:
                return e
        return None

    def _on_detections(self, msg: Detection2DArray) -> None:
        m = self.mission
        if m is None or m['phase'] != 'observe' or not self._fresh(msg.header.stamp):
            return
        if self.now() - m['t_obs'] < float(self.ip['observe_settle']):
            return
        m['frames'] += 1
        e: WatchEntry = m['entry']
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id != self.ip['lying_class']:
                continue
            loc = self._locate(det)
            if loc is not None and self.watch.distance(e, loc[0], loc[1]) <= \
                    float(self.ip['merge_radius']):
                m['hits'].append((loc[0], loc[1], det.results[0].hypothesis.score))
                m.setdefault('cluster', []).extend(loc[4])
                if self._last_posture is not None:
                    m['posture'] = self._last_posture
                break

    def _on_posture(self, msg: String) -> None:
        try:
            self._last_posture = json.loads(msg.data)
        except ValueError:
            self._last_posture = None

    def _on_mission_image(self, msg: Image) -> None:
        self._last_image = msg

    # ------------------------------------------------------------------ görev makinesi
    def _viewpoints(self, e: WatchEntry) -> List[tuple]:
        robot = self._robot_pose(timeout=0.0)
        if self._grid is None or robot is None:
            return []
        # yasak bölge costmap'te engeldir: görüş hattı denetimi onu (ve şişirmesini) yok sayar
        clear = max(0.7, float(self.ip['keepout_radius']) + 0.4)
        return geo.plan_viewpoints(self._grid, e.x, e.y, float(self.ip['inspect_radius']),
                                   int(self.ip['inspect_views']), robot[:2], clear_radius=clear)

    def _start_mission(self, kind: str, e: WatchEntry) -> None:
        vps = self._viewpoints(e)
        if kind == 'revisit':
            # önce daha önce kişinin görüldüğü açılar, sonra yeni planlananlar
            seen = [tuple(o['viewpoint']) for o in e.observations
                    if o.get('seen') and o.get('viewpoint')]
            extra = [v for v in vps if all(math.hypot(v[0] - s[0], v[1] - s[1]) > 0.5
                                           for s in seen)]
            vps = seen + extra
        self.mission = {'kind': kind, 'entry': e, 'vps': vps, 'i': -1, 'phase': 'cancelling',
                        't0': self.now(), 'results': [], 'handle': None}
        if self.ip['alert_image_topic'] and self._img_sub is None:
            self._img_sub = self.create_subscription(Image, str(self.ip['alert_image_topic']),
                                                     self._on_mission_image, 1)
        if not vps:
            self.get_logger().warning(f'#{e.id}: bakış noktası planlanamadı (costmap yok ya da '
                                      'çevre kapalı)' + (' — bulunduğum yerden kayıt alınıyor'
                                                         if kind == 'inspect' else ''))
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()      # sonucu _on_result → _mission_step
        elif self.goal_idx is None:
            self._mission_step()
        # aksi halde hedef gönderildi ama henüz kabul edilmedi: _on_accept iptal eder

    def _mission_step(self) -> None:
        m = self.mission
        m['i'] += 1
        if m['i'] >= len(m['vps']):
            if not m['vps'] and m['kind'] == 'inspect' and not m['results']:
                self._begin_observe(None)   # yerinde tek gözlem
                return
            self._mission_finish()
            return
        vp = m['vps'][m['i']]
        m['phase'], m['goal_start'] = 'nav', self.now()
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.frame_id
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = vp[0], vp[1]
        g.pose.pose.orientation.z = math.sin(vp[2] / 2)
        g.pose.pose.orientation.w = math.cos(vp[2] / 2)
        what = 'kayıt' if m['kind'] == 'inspect' else 'revisit'
        self.get_logger().info(f'#{m["entry"].id} {what}: bakış noktası {m["i"] + 1}/'
                               f'{len(m["vps"])} ({vp[0]:+.2f}, {vp[1]:+.2f})')
        self.nav.send_goal_async(g).add_done_callback(self._on_mission_accept)

    def _on_mission_accept(self, fut) -> None:
        gh = fut.result()
        if self.mission is None:
            return
        if not gh.accepted:
            self._viewpoint_failed('hedef reddedildi')
            return
        self.mission['handle'] = gh
        gh.get_result_async().add_done_callback(self._on_mission_result)

    def _on_mission_result(self, fut) -> None:
        m = self.mission
        if m is None:
            return
        m['handle'] = None
        if fut.result().status == GoalStatus.STATUS_SUCCEEDED:
            self._begin_observe(m['vps'][m['i']])
        else:
            self._viewpoint_failed(f'Nav2 durum={fut.result().status}')

    def _viewpoint_failed(self, why: str) -> None:
        m = self.mission
        vp = m['vps'][m['i']]
        m['results'].append({'viewpoint': [round(v, 3) for v in vp], 'reached': False,
                             'reason': why})
        self.get_logger().warning(f'#{m["entry"].id}: bakış noktasına ulaşılamadı ({why})')
        self._mission_step()

    def _begin_observe(self, vp: Optional[tuple]) -> None:
        m = self.mission
        m.update({'phase': 'observe', 't_obs': self.now(), 'frames': 0, 'hits': [],
                  'posture': None, 'vp': vp})
        self._last_image = None

    def _mission_tick(self) -> None:
        m = self.mission
        if m is None:
            return
        now = self.now()
        if m['phase'] == 'observe' and now - m['t_obs'] >= float(self.ip['observe_time']):
            self._observe_done()
        elif m['phase'] == 'nav' and m['handle'] is not None and \
                now - m['goal_start'] > self.goal_timeout:
            m['handle'].cancel_goal_async()   # sonuç "ulaşılamadı" olarak işlenir

    def _save_image(self, tag: str) -> Optional[str]:
        msg = self._last_image
        if msg is None or msg.encoding not in ('bgr8', 'rgb8'):
            return None
        try:
            os.makedirs(self.watch.dir, exist_ok=True)
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                a = a[..., ::-1]
            path = os.path.join(self.watch.dir, f'{tag}_{time.strftime("%Y%m%d_%H%M%S")}.jpg')
            PilImage.fromarray(np.ascontiguousarray(a)).save(path, quality=90)
            return path
        except Exception as e:  # noqa: BLE001 - kayıt hatası devriyeyi düşürmesin
            self.get_logger().error(f'kare kaydedilemedi: {e!r}')
            return None

    def _observe_done(self) -> None:
        m = self.mission
        e: WatchEntry = m['entry']
        hits = m['hits']
        seen = len(hits) >= int(self.ip['observe_min_hits'])
        robot = self._robot_pose(timeout=0.0)
        kind = m['kind']
        n_obs = len(e.observations) + len(e.revisits) + 1
        image = self._save_image(f'wl{e.id:03d}_{kind}{n_obs:02d}')
        rec = {'sim_time': round(self.now(), 3),
               'viewpoint': None if m['vp'] is None else [round(v, 3) for v in m['vp']],
               'robot_map_pose': None if robot is None else [round(v, 3) for v in robot],
               'reached': True, 'frames': m['frames'], 'lying_hits': len(hits),
               'max_score': round(max((h[2] for h in hits), default=0.0), 3), 'seen': seen,
               'image': image, 'posture': m['posture']}
        if image:
            e.images.append(image)
        if seen:
            e.last_seen_sim = rec['sim_time']
        m['results'].append(rec)
        m['phase'] = 'idle'
        if kind == 'inspect':
            m.setdefault('all_hits', []).extend(hits)
            e.observations.append(rec)
            self.watch.log_event('observation', id=e.id, **rec)
            self.get_logger().info(
                f'#{e.id} kayıt {len(e.observations)}: {len(hits)}/{m["frames"]} karede yatan '
                f'insan, kare: {image}')
            self._mission_step()
            return
        # revisit kararı
        if seen:
            e.confirmations += 1
            e.notifications += 1
            rec['result'] = 'still_lying'
            self._close_revisit(rec, float(self.ip['revisit_interval']))
            self.get_logger().warning(
                f'!!! YATAN İNSAN #{e.id} HÂLÂ YERDE ({e.x:+.2f}, {e.y:+.2f}): doğrulama '
                f'{e.confirmations}, bildirim {e.notifications} ({len(hits)}/{m["frames"]} kare)')
            self._mission_finish(done=True)
            return
        view = self._check_view(e, robot)
        rec['view'] = view
        if view is not None and view['clear']:
            rec['result'] = 'resolved'
            self.watch.resolve(e)
            e.revisits.append(rec)
            self.total['revisits'] += 1
            self.total['resolved'] += 1
            self.watch.log_event('resolved', id=e.id, **rec)
            self.get_logger().info(
                f'ÇÖZÜLDÜ: #{e.id} ({e.x:+.2f}, {e.y:+.2f}) artık yok — konum görüş alanında '
                f'({view["cam_dist_m"]} m, {view["cam_bearing_deg"]}°), görüş hattı açık, konum '
                f'boş; listeden çıkarıldı')
            self._mission_finish(done=True)
            return
        rec['result'] = 'view_unverified'
        m['unverified'] = m.get('unverified', []) + [rec]
        self.get_logger().info(f'#{e.id}: kişi görülmedi ama görüş doğrulanamadı ({view}); '
                               'listeden ÇIKARILMADI, başka açı deneniyor')
        self._mission_step()

    def _check_view(self, e: WatchEntry, robot: Optional[tuple]) -> Optional[dict]:
        frames, cam = self._sensor_frames(), self._cam
        if frames is None or cam is None or robot is None or self._scan is None:
            return None
        cam_b, lid_b = frames
        hfov = 2.0 * math.atan2(cam.width / 2.0, cam.k[0])
        return geo.check_view(self._scan_pts(lid_b, keep_far=True), robot, cam_b[:3], lid_b[:3],
                              e.x, e.y, hfov, float(self.ip['view_max_range']))

    def _close_revisit(self, rec: dict, delay: float) -> None:
        e: WatchEntry = self.mission['entry']
        e.revisits.append(rec)
        e.next_revisit_sim = round(self.now() + delay, 3)
        self.total['revisits'] += 1
        self.total['confirmed' if rec['result'] == 'still_lying' else 'unverified'] += 1
        self.watch.log_event('revisit', id=e.id, **rec)

    def _mission_finish(self, done: bool = False) -> None:
        m = self.mission
        e: WatchEntry = m['entry']
        if m['kind'] == 'inspect':
            pts = m.get('all_hits', [])
            if pts:   # konumu tüm açılardaki ölçümlerle iyileştir
                mx = float(np.median([p[0] for p in pts]))
                my = float(np.median([p[1] for p in pts]))
                # gövde noktaları: farklı açılardan görülen kümelerin birleşimi (aykırılar hariç)
                body = [p for p in m.get('cluster', []) if math.hypot(p[0] - mx, p[1] - my) <= 1.3]
                self.watch.set_extent(e, body)
                if len(e.extent) >= 3:   # merkez = görülen gövdenin sınır kutusu ortası
                    ex = np.asarray(e.extent)
                    mx, my = (0.5 * (ex.min(axis=0) + ex.max(axis=0))).tolist()
                e.x, e.y = round(mx, 3), round(my, 3)
            e.next_revisit_sim = round(self.now() + float(self.ip['revisit_interval']), 3)
            n_seen = sum(1 for o in e.observations if o['seen'])
            self.get_logger().info(
                f'#{e.id} kaydı tamam: {len(e.observations)} açı ({n_seen} açıda görüldü), '
                f'{len(e.images)} kare; revisit {self.ip["revisit_interval"]:.0f} sim-s sonra. '
                'Devriye sürüyor')
        elif not done:
            reasons = m.get('unverified', []) or m['results']
            rec = {'sim_time': round(self.now(), 3), 'result': 'view_unverified',
                   'attempts': len(m['results']),
                   'detail': [{k: r.get(k) for k in ('viewpoint', 'reached', 'reason', 'view')}
                              for r in reasons]}
            self._close_revisit(rec, float(self.ip['revisit_retry']))
            self.get_logger().warning(
                f'#{e.id} revisit sonuçsuz: görüş doğrulanamadı ({len(m["results"])} deneme); '
                f'kayıt listede kalıyor, {self.ip["revisit_retry"]:.0f} sim-s sonra yeniden')
        self.watch.save()
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        self.mission = None
        self._publish_markers()
        if self.goal_idx is not None:
            self._send_goal(self.goal_idx)   # yarım kalan devriye hedefi
        else:
            self._next()

    def _maybe_revisit(self) -> bool:
        """_next() başında çağrılır: zamanı gelen kayıt varsa revisit görevini başlatır."""
        if not bool(self.ip['react_to_alerts']) or self.mission is not None:
            return False
        e = self.watch.due(self.now())
        if e is None:
            return False
        self.get_logger().info(f'#{e.id} için revisit zamanı ({e.x:+.2f}, {e.y:+.2f})')
        self._start_mission('revisit', e)
        return True

    # ------------------------------------------------------------------ yasak bölge
    def _publish_keepout(self) -> None:
        """
        Etkin kayıtların çevresini Nav2 costmap'lerine engel olarak bildirir (PointCloud2, map).

        LiDAR düzlemi (zeminden ~11 cm) yatan gövdenin yalnızca bir kısmını görür; kol/bacak gibi
        görülmeyen uzantılara sürmemek için kişinin çevresi `keepout_radius` kadar kapatılır.
        Kayıt kalmayınca boş bulut yayınlanır (costmap'teki son gözlem silinsin).
        """
        r = float(self.ip['keepout_radius'])
        discs = [(e.x, e.y, r) for e in self.watch.active()]
        if not discs and not self._keepout_used:
            return
        self._keepout_used = True
        robot = self._robot_pose(timeout=0.0)
        pts = geo.keepout_points(discs, None if robot is None else robot[:2])
        header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
        xyz = np.column_stack([pts, np.full(len(pts), 0.1)]).astype(np.float32)
        self._keepout_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz))

    # ------------------------------------------------------------------ RViz
    def _watch_markers(self) -> List[Marker]:
        out: List[Marker] = []
        for k, e in enumerate(self.watch.entries[-30:]):
            active = e.status == 'active'
            label = (f'YATAN #{e.id} (doğrulama {e.confirmations})' if active
                     else f'#{e.id} çözüldü')
            for ns, mtype, z, size in (('alert', Marker.SPHERE, 0.15, 0.35 if active else 0.2),
                                       ('alert_label', Marker.TEXT_VIEW_FACING, 0.8, 0.3)):
                mk = Marker()
                mk.header.frame_id = self.frame_id
                mk.ns, mk.id, mk.type, mk.action = ns, k, mtype, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = e.x, e.y, z
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = size
                mk.color.r, mk.color.g = (1.0, 0.0) if active else (0.2, 0.8)
                mk.color.b, mk.color.a = 0.1, 0.9
                mk.text = label
                out.append(mk)
        return out
