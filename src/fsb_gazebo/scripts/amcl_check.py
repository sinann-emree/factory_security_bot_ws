#!/usr/bin/env python3
"""
AMCL yakınsama testi (Aşama 5 çıkış kriteri).

1) Robotun gerçek pozuna göre KASITLI HATALI bir /initialpose yayınlanır
   (varsayılan +0.5 m, -0.4 m, +20°).
2) Robot /cmd_vel_nav üzerinden (velocity_smoother → collision_monitor zinciri) yerinde döndürülür,
   sonra kısa ileri-geri sürülür; AMCL her güncellemede parçacıkları yeniden örnekler.
3) Her sim-saniyede: AMCL tahmini – gerçek konum hatası, kovaryans σ ve parçacık bulutu
   yayılımı loglanır.
Yakınsama ölçütü: konum hatası < 0.10 m ve yaw hatası < 3° ve parçacık bulutu σ_xy başlangıca göre
en az %50 daralmış. (Nav2 likelihood_field modeli, sigma_hit 0.2 / z_rand 0.5 ile, yakınsamış
durumda da ~20-25 cm bulut σ'sı korur; mutlak σ eşiği bu yüzden kullanılmaz.)
Hata, son AMCL tahmininin zaman damgasındaki gerçek pozla hesaplanır.

Varsayım: map çerçevesi = dünya çerçevesi (harita SLAM ile (0,0,0)'dan başlatılarak çıkarıldı).
Kullanım (bringup localization:=amcl çalışırken):
  ros2 run fsb_gazebo amcl_check.py [--dx 0.5 --dy -0.4 --dyaw 20]
"""

import argparse
import bisect
from collections import deque
import math
import sys
import threading
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_msgs.msg import ParticleCloud
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock

Pose2D = Tuple[float, float, float]


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AmclCheck(Node):

    def __init__(self, world: str, model: str) -> None:
        super().__init__('fsb_amcl_check',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.model = model
        self.gt: Optional[Pose2D] = None
        self._gt_hist: deque = deque(maxlen=4000)  # (sim zamanı, poz)
        self._gt_lock = threading.Lock()
        self.amcl: Optional[Pose2D] = None
        self.amcl_t = 0.0
        self.amcl_cov: Optional[np.ndarray] = None
        self.amcl_count = 0
        self.spread: Optional[Tuple[float, float, int]] = None  # σ_xy, σ_yaw, N
        self.sim_t: Optional[float] = None
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        best_effort = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ParticleCloud, '/particle_cloud', self._on_cloud, best_effort)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self._on_gt):
            raise RuntimeError('gz-transport aboneliği başarısız')

    def _on_clock(self, m: Clock) -> None:
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_gt(self, m: Pose_V) -> None:
        for ps in m.pose:
            if ps.name == self.model:
                self.gt = (ps.position.x, ps.position.y, yaw_of(ps.orientation))
                with self._gt_lock:
                    self._gt_hist.append(
                        (m.header.stamp.sec + m.header.stamp.nsec * 1e-9, self.gt))

    def gt_at(self, t: float) -> Pose2D:
        """Döndür: t sim-zamanına en yakın gerçek poz (AMCL tahmini o ana aittir)."""
        with self._gt_lock:
            hist = list(self._gt_hist)
        i = bisect.bisect_left([h[0] for h in hist], t)
        cand = [hist[j] for j in (i - 1, i) if 0 <= j < len(hist)]
        return min(cand, key=lambda h: abs(h[0] - t))[1] if cand else self.gt

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.amcl_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.amcl_cov = np.array(m.pose.covariance).reshape(6, 6)
        self.amcl_count += 1

    def _on_cloud(self, m: ParticleCloud) -> None:
        if not m.particles:
            return
        xy = np.array([[pt.pose.position.x, pt.pose.position.y] for pt in m.particles])
        yaw = np.array([yaw_of(pt.pose.orientation) for pt in m.particles])
        w = np.array([pt.weight for pt in m.particles])
        w = w / w.sum() if w.sum() > 0 else np.full(len(w), 1.0 / len(w))
        mu = (w[:, None] * xy).sum(0)
        s_xy = math.sqrt(float((w * ((xy - mu) ** 2).sum(1)).sum()))
        cy, sy = float((w * np.cos(yaw)).sum()), float((w * np.sin(yaw)).sum())
        r = min(1.0, math.hypot(cy, sy))
        s_yaw = math.sqrt(max(0.0, -2.0 * math.log(max(r, 1e-12))))  # dairesel σ
        self.spread = (s_xy, s_yaw, len(m.particles))

    # --- yardımcılar ---
    def spin_sim(self, seconds: float, v: float = 0.0, w: float = 0.0) -> None:
        t0 = self.sim_t
        while self.sim_t - t0 < seconds:
            if v or w:
                self.cmd(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)

    def cmd(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub_cmd.publish(msg)

    def error(self) -> Tuple[float, float]:
        g = self.gt_at(self.amcl_t)  # son AMCL tahmininin damgasındaki gerçek poz
        ex = math.hypot(self.amcl[0] - g[0], self.amcl[1] - g[1])
        return ex, abs(math.degrees(wrap(self.amcl[2] - g[2])))

    def row(self, tag: str, rows: List[str]) -> Tuple[float, float, float]:
        ex, eyaw = self.error()
        sxy, syaw, n = self.spread if self.spread else (float('nan'),) * 2 + (0,)
        cov_xy = math.sqrt(max(self.amcl_cov[0, 0], self.amcl_cov[1, 1]))
        line = (f'{tag:>12s} t={self.sim_t:7.1f}  hata={ex * 100:6.1f} cm {eyaw:5.1f}°  '
                f'kov σ={cov_xy * 100:5.1f} cm  bulut σxy={sxy * 100:5.1f} cm '
                f'σyaw={math.degrees(syaw):4.1f}° N={n}')
        rows.append(line)
        self.get_logger().info(line)
        return ex, eyaw, sxy


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--dx', type=float, default=0.5)
    ap.add_argument('--dy', type=float, default=-0.4)
    ap.add_argument('--dyaw', type=float, default=20.0, help='derece')
    args = ap.parse_args(argv)

    rclpy.init()
    n = AmclCheck(args.world, args.model)
    log = n.get_logger()
    rows: List[str] = []
    try:
        t0 = time.monotonic()
        while n.gt is None or n.sim_t is None or n.amcl is None:
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.monotonic() - t0 > 30.0:
                raise RuntimeError(f'veri yok: gt={n.gt} clock={n.sim_t} amcl={n.amcl}')
        gx, gy, gyaw = n.gt
        init = PoseWithCovarianceStamped()
        init.header.frame_id = 'map'
        init.header.stamp = n.get_clock().now().to_msg()
        yaw0 = gyaw + math.radians(args.dyaw)
        init.pose.pose.position.x = gx + args.dx
        init.pose.pose.position.y = gy + args.dy
        init.pose.pose.orientation.z = math.sin(yaw0 / 2.0)
        init.pose.pose.orientation.w = math.cos(yaw0 / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.5 ** 2   # σ = 0.5 m
        cov[35] = math.radians(30) ** 2  # σ = 30°
        init.pose.covariance = cov
        log.info(f'Gerçek poz ({gx:+.2f}, {gy:+.2f}, {math.degrees(gyaw):+.1f}°); '
                 f'hatalı initialpose: +({args.dx}, {args.dy}) m, +{args.dyaw}°')
        c0 = n.amcl_count
        for _ in range(5):
            n.pub_init.publish(init)
            n.spin_sim(0.2)
            if n.amcl_count > c0:
                break
        n.spin_sim(1.0)
        e0 = n.row('başlangıç', rows)

        conv_t: Optional[float] = None
        start_t = n.sim_t
        # ileri ve geri aynı yön/hız/süre → robot başladığı yere döner (tekrarlanabilir test)
        phases = [('dön +', 0.0, 0.6, 12.0), ('ileri', 0.15, 0.0, 6.0),
                  ('geri', -0.15, 0.0, 6.0), ('dön -', 0.0, -0.6, 12.0),
                  ('bekle', 0.0, 0.0, 3.0)]
        for name, v, w, dur in phases:
            k = 0
            while k < dur:
                n.spin_sim(1.0, v, w)
                k += 1
                ex, eyaw, sxy = n.row(name, rows)
                if conv_t is None and ex < 0.10 and eyaw < 3.0 and sxy < 0.5 * e0[2]:
                    conv_t = n.sim_t - start_t
        n.spin_sim(0.5)
        ex, eyaw, sxy = n.row('SON', rows)
        ok = ex < 0.10 and eyaw < 3.0 and sxy < 0.5 * e0[2]
        log.info('=' * 70)
        log.info(f'Başlangıç hatası: {e0[0] * 100:.1f} cm / {e0[1]:.1f}°  '
                 f'(bulut σxy {e0[2] * 100:.1f} cm)')
        log.info(f'Son hata:        {ex * 100:.1f} cm / {eyaw:.1f}°  '
                 f'(bulut σxy {sxy * 100:.1f} cm, daralma %{100 * (1 - sxy / e0[2]):.0f})')
        log.info(f'Yakınsama süresi: {conv_t:.1f} sim-s' if conv_t is not None
                 else 'Yakınsama süresi: ölçüt hiç sağlanmadı')
        log.info(f'AMCL güncellemesi: {n.amcl_count - c0}  SONUÇ: {"GEÇTİ" if ok else "KALDI"}')
        return 0 if ok else 1
    finally:
        n.cmd(0.0, 0.0)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
