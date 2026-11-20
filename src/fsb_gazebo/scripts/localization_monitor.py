#!/usr/bin/env python3
"""
Lokalizasyon izleyici: map→odom sıçramaları ve map→base_footprint doğruluğu.

Aşama 5 çıkış kriteri.

10 Hz (sim saati) ile TF'ten okunur:
  * map→odom ham adımı: ardışık örnekler arası değişim (Δxy, Δyaw). Not: odom orijini robottan
    uzaksa küçük bir yaw düzeltmesi bile büyük Δxy üretir (kaldıraç: r·Δθ).
  * POZ SIÇRAMASI (asıl ölçüt): map→odom değişiminin robotun bulunduğu noktada yarattığı yer
    değiştirme |T_yeni·p − T_eski·p| (p = odom→base_footprint). Eşik üstü = "zıplama".
  * map→base_footprint ile Gazebo gerçek konumu arasındaki hata (map = dünya varsayımı).
  * Gerçek konumdan kat edilen yol.
Ctrl+C (SIGINT) veya --duration (sim-s) dolunca özet basılır; her --report sim-s'de ara özet.
Kullanım:
  ros2 run fsb_gazebo localization_monitor.py [--duration 0] [--jump-xy 0.10 --jump-yaw 3]
"""

import argparse
import bisect
from collections import deque
import math
import sys
import threading
from typing import List, Optional, Tuple

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

Pose2D = Tuple[float, float, float]


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def apply(t: Pose2D, p: Pose2D) -> Tuple[float, float]:
    """Uygula: 2B dönüşüm t'yi p noktasına."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return (t[0] + c * p[0] - s * p[1], t[1] + s * p[0] + c * p[1])


class Monitor(Node):

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('fsb_localization_monitor',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.gt: Optional[Pose2D] = None
        # (sim zamanı, poz) geçmişi: TF damgasındaki gerçek pozla karşılaştırmak için
        self._gt_hist: deque = deque(maxlen=4000)
        self._gt_lock = threading.Lock()
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', self._on_gt):
            raise RuntimeError('gz-transport aboneliği başarısız')
        self.prev_mo: Optional[Pose2D] = None
        self.prev_gt: Optional[Pose2D] = None
        self.steps_xy: List[float] = []
        self.steps_yaw: List[float] = []
        self.pose_jumps: List[float] = []
        self.err_xy: List[float] = []
        self.err_yaw: List[float] = []
        self.jumps: List[str] = []
        self.path = 0.0
        self.t_start: Optional[float] = None
        self.t_last_report = 0.0
        self.done = False
        self.create_timer(0.1, self._tick)

    def _on_gt(self, m: Pose_V) -> None:
        for ps in m.pose:
            if ps.name == self.args.model:
                self.gt = (ps.position.x, ps.position.y, yaw_of(ps.orientation))
                t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                with self._gt_lock:
                    self._gt_hist.append((t, self.gt))

    def _gt_at(self, t: float) -> Optional[Pose2D]:
        with self._gt_lock:
            hist = list(self._gt_hist)
        if not hist:
            return None
        i = bisect.bisect_left([h[0] for h in hist], t)
        best = min((hist[j] for j in (i - 1, i) if 0 <= j < len(hist)),
                   key=lambda h: abs(h[0] - t))
        return best[1] if abs(best[0] - t) < 0.05 else None

    def _lookup(self, parent: str, child: str) -> Optional[Tuple[Pose2D, float]]:
        try:
            t = self.buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.0))
        except Exception:  # noqa: BLE001 — TF henüz yok
            return None
        tr = t.transform
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation)), stamp

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        mo_s = self._lookup('map', 'odom')
        mb_s = self._lookup('map', 'base_footprint')
        ob_s = self._lookup('odom', 'base_footprint')
        if mo_s is None or mb_s is None or ob_s is None:
            return
        ob = ob_s[0]
        mo, (mb, mb_t) = mo_s[0], mb_s
        gt = self._gt_at(mb_t)  # TF zincirinin damgasındaki gerçek poz (gecikme hatasını önler)
        if gt is None:
            return
        if self.t_start is None:
            self.t_start = self.t_last_report = now
        if self.prev_mo is not None:
            dxy = math.hypot(mo[0] - self.prev_mo[0], mo[1] - self.prev_mo[1])
            dyaw = abs(math.degrees(wrap(mo[2] - self.prev_mo[2])))
            self.steps_xy.append(dxy)
            self.steps_yaw.append(dyaw)
            pj = math.dist(apply(mo, ob), apply(self.prev_mo, ob))
            self.pose_jumps.append(pj)
            if pj > self.args.jump_xy or dyaw > self.args.jump_yaw:
                msg = (f't={now:.1f} poz sıçraması={pj * 100:.1f} cm Δyaw={dyaw:.2f}° '
                       f'(map→odom ham Δxy={dxy * 100:.1f} cm)')
                self.jumps.append(msg)
                self.get_logger().warn(f'SIÇRAMA: {msg}')
        self.prev_mo = mo
        self.err_xy.append(math.hypot(mb[0] - gt[0], mb[1] - gt[1]))
        self.err_yaw.append(abs(math.degrees(wrap(mb[2] - gt[2]))))
        if self.prev_gt is not None:
            self.path += math.hypot(gt[0] - self.prev_gt[0], gt[1] - self.prev_gt[1])
        self.prev_gt = gt
        if now - self.t_last_report >= self.args.report:
            self.t_last_report = now
            self.summary('ARA ÖZET')
        if self.args.duration > 0 and now - self.t_start >= self.args.duration:
            self.done = True

    def summary(self, title: str) -> None:
        log = self.get_logger()
        if not self.err_xy:
            log.info(f'{title}: veri yok')
            return
        sx, sy = np.array(self.steps_xy or [0.0]), np.array(self.steps_yaw or [0.0])
        pj = np.array(self.pose_jumps or [0.0])
        ex, ey = np.array(self.err_xy), np.array(self.err_yaw)
        dur = self.get_clock().now().nanoseconds * 1e-9 - self.t_start
        log.info(f'--- {title}: {dur:.0f} sim-s, {len(ex)} örnek, yol {self.path:.1f} m ---')
        log.info(f'map→odom ham adım: maks Δxy={sx.max() * 100:.2f} cm  '
                 f'maks Δyaw={sy.max():.2f}°  '
                 f'p99 Δxy={np.percentile(sx, 99) * 100:.2f} cm')
        log.info(f'POZ SIÇRAMASI (robot noktasında): maks={pj.max() * 100:.2f} cm  '
                 f'p99={np.percentile(pj, 99) * 100:.2f} cm  '
                 f'sıçrama(>{self.args.jump_xy * 100:.0f} cm / >{self.args.jump_yaw}°)='
                 f'{len(self.jumps)}')
        log.info(f'map→base_footprint vs gerçek: xy ort={ex.mean() * 100:.1f} '
                 f'p95={np.percentile(ex, 95) * 100:.1f} maks={ex.max() * 100:.1f} cm   '
                 f'yaw ort={ey.mean():.2f} p95={np.percentile(ey, 95):.2f} maks={ey.max():.2f}°')


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--duration', type=float, default=0.0, help="sim-s, 0 = Ctrl+C'ye kadar")
    ap.add_argument('--report', type=float, default=120.0, help='ara özet aralığı (sim-s)')
    ap.add_argument('--jump-xy', type=float, default=0.10, help='sıçrama eşiği (m)')
    ap.add_argument('--jump-yaw', type=float, default=3.0, help='sıçrama eşiği (°)')
    args = ap.parse_args(argv)

    rclpy.init()
    n = Monitor(args)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary('SON ÖZET')
        for j in n.jumps[:20]:
            n.get_logger().info(f'  sıçrama: {j}')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
