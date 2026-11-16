#!/usr/bin/env python3
"""
Sürüş / drift testi: tekerlek odometrisi ve EKF tahminini Gazebo gerçek konumuyla karşılaştırır.

Gerçek konum (ground truth) doğrudan gz-transport ile okunur:
  /world/<world>/dynamic_pose/info  (gz.msgs.Pose_V, model adı ile)
Bu test aracıdır; köprü yaml'ına ground truth eklenmez.

Modlar:
  basic : 1 m ileri, +360°, -360° (Aşama 3 çıkış kriteri)
  drift : --duration sim-saniye boyunca serbest alanda rastgele hedeflere sürüş (+ ara sıra yerinde
          dönüş), sonra EKF tahminine göre başlangıç pozuna dönüş; gerçek konumdaki kapanma hatası
          ölçülür (Aşama 4 çıkış kriteri)

Kullanım (simülasyon çalışırken):
  ros2 run fsb_gazebo drive_test.py basic
  ros2 run fsb_gazebo drive_test.py drift --duration 300 --seed 1
"""

import argparse
import math
import random
import sys
import time
from typing import Callable, Optional, Tuple

from geometry_msgs.msg import TwistStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

Pose2D = Tuple[float, float, float]  # x, y, yaw (yaw sürekli/unwrap edilmiş)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Unwrap:
    """Açıyı ±π sıçramalarından arındırıp sürekli toplar."""

    def __init__(self) -> None:
        self.prev: Optional[float] = None
        self.acc = 0.0

    def __call__(self, a: float) -> float:
        if self.prev is None:
            self.acc = a
        else:
            self.acc += wrap(a - self.prev)
        self.prev = a
        return self.acc


def relative(p0: Pose2D, p: Pose2D) -> Pose2D:
    """Göreli poz: p, p0 gövde çerçevesinde."""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, p[2] - p0[2])


class DriveTester(Node):

    def __init__(self, world: str, model: str, cmd_topic: str) -> None:
        super().__init__('fsb_drive_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.model = model
        self.odom: Optional[Pose2D] = None
        self.ekf: Optional[Pose2D] = None
        self.gt: Optional[Pose2D] = None
        self.gt_z = 0.0
        self.sim_t: Optional[float] = None
        self._u_odom, self._u_ekf, self._u_gt = Unwrap(), Unwrap(), Unwrap()
        self.pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self._on_odom, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 50)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._gz = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self._gz.subscribe(Pose_V, topic, self._on_gt):
            raise RuntimeError(f'gz-transport aboneliği başarısız: {topic}')

    # --- geri çağırmalar ---
    def _on_clock(self, m: Clock) -> None:
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, self._u_odom(yaw_of(p.orientation)))

    def _on_ekf(self, m: Odometry) -> None:
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, self._u_ekf(yaw_of(p.orientation)))

    def _on_gt(self, m: Pose_V) -> None:  # gz-transport iş parçacığı
        for ps in m.pose:
            if ps.name == self.model:
                self.gt = (ps.position.x, ps.position.y, self._u_gt(yaw_of(ps.orientation)))
                self.gt_z = ps.position.z

    # --- yardımcılar ---
    def wait_ready(self, need_ekf: bool, timeout: float = 30.0) -> None:
        t0 = time.monotonic()
        while (self.odom is None or self.gt is None or self.sim_t is None
               or (need_ekf and self.ekf is None)):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(
                    f'veri yok: odom={self.odom is not None} gt={self.gt is not None} '
                    f'clock={self.sim_t is not None} ekf={self.ekf is not None}')

    def send(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)

    def spin_for(self, sim_seconds: float, v: float = 0.0, w: float = 0.0) -> None:
        t0 = self.sim_t
        while self.sim_t - t0 < sim_seconds:
            self.send(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive_until(self, v: float, w: float, done: Callable[[], bool],
                    timeout: float = 120.0) -> None:
        t0 = self.sim_t
        while not done():
            self.send(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_t - t0 > timeout:
                raise RuntimeError('drive_until zaman aşımı')
        self.spin_for(1.5)  # dur ve otur

    def control_step(self, est: Callable[[], Pose2D], tx: float, ty: float,
                     tyaw: Optional[float], tol: float = 0.05) -> bool:
        """Tahmini poza göre hedefe bir kontrol adımı; varınca True."""
        x, y, yaw = est()
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist > tol:
            e = wrap(math.atan2(dy, dx) - yaw)
            w = max(-0.8, min(0.8, 2.0 * e))
            v = 0.0 if abs(e) > 0.3 else max(0.02, min(0.2, 0.6 * dist))
            self.send(v, w)
            return False
        if tyaw is not None:
            e = wrap(tyaw - yaw)
            if abs(e) > 0.005:
                self.send(0.0, max(-0.5, min(0.5, 2.0 * e)) if abs(e) > 0.05
                          else math.copysign(0.05, e))
                return False
        self.send(0.0, 0.0)
        return True


def run_basic(n: DriveTester) -> None:
    log = n.get_logger()
    n.wait_ready(need_ekf=False)
    n.spin_for(1.0)
    wall0, sim0 = time.monotonic(), n.sim_t

    o0, g0 = n.odom, n.gt
    n.drive_until(0.15, 0.0, lambda: math.hypot(n.odom[0] - o0[0], n.odom[1] - o0[1]) >= 1.0)
    o1, g1 = n.odom, n.gt
    ro, rg = relative(o0, o1), relative(g0, g1)
    d_o, d_g = math.hypot(ro[0], ro[1]), math.hypot(rg[0], rg[1])
    log.info(f'T1 1 m ileri: odom={d_o:.4f} m  gerçek={d_g:.4f} m  fark={d_g - d_o:+.4f} m '
             f'({100 * (d_g - d_o) / d_o:+.2f}%)  yanal={rg[1]:+.4f} m  '
             f'Δyaw gerçek={math.degrees(rg[2]):+.3f}°  odom={math.degrees(ro[2]):+.3f}°')

    for key, w in (('T2 +360°', 0.5), ('T3 -360°', -0.5)):
        o0, g0 = n.odom, n.gt
        n.drive_until(0.0, w, lambda: abs(n.odom[2] - o0[2]) >= 2 * math.pi)
        o1, g1 = n.odom, n.gt
        log.info(f'{key} dönüş: odom Δyaw={o1[2] - o0[2]:+.4f} rad  '
                 f'gerçek Δyaw={g1[2] - g0[2]:+.4f} rad (2π=6.2832)  '
                 f'fark={math.degrees((g1[2] - g0[2]) - (o1[2] - o0[2])):+.3f}°  '
                 f'merkez kayması={math.hypot(g1[0] - g0[0], g1[1] - g0[1]):.4f} m')
    log.info(f'RTF={(n.sim_t - sim0) / (time.monotonic() - wall0):.3f}  gövde z={n.gt_z:.2e} m')


def run_drift(n: DriveTester, duration: float, seed: int, box_x: float, box_y: float) -> None:
    log = n.get_logger()
    rng = random.Random(seed)
    n.wait_ready(need_ekf=True)
    n.spin_for(1.0)
    if abs(n.gt[0]) > 1.0 or abs(n.gt[1]) > 0.5:
        log.warn(f'Başlangıç ana koridor ortasında değil (gt={n.gt[:2]}); geofence dünya '
                 'koordinatında güvenli olmayabilir.')
    e0, o0, g0 = n.ekf, n.odom, n.gt
    wall0, sim0 = time.monotonic(), n.sim_t
    path, rot = 0.0, 0.0
    last = g0
    # Hareket halinde maks. hata yayın gecikmesini de içerir (EKF 30 Hz, odom 50 Hz);
    # drift için asıl ölçü duraklama anlarındaki hata (still_err) ve son hatadır.
    max_err = {'ekf_xy': 0.0, 'ekf_yaw': 0.0, 'odom_xy': 0.0, 'odom_yaw': 0.0}
    still_err = dict(max_err)
    targets, spins = 0, 0

    def errors() -> dict:
        rg = relative(g0, n.gt)
        out = {}
        for key, est, est0 in (('ekf', n.ekf, e0), ('odom', n.odom, o0)):
            r = relative(est0, est)
            out[f'{key}_xy'] = math.hypot(r[0] - rg[0], r[1] - rg[1])
            out[f'{key}_yaw'] = wrap(r[2] - rg[2])
        return out

    def ekf_rel() -> Pose2D:
        return relative(e0, n.ekf)

    def track() -> None:
        nonlocal path, rot, last
        g = n.gt
        path += math.hypot(g[0] - last[0], g[1] - last[1])
        rot += abs(g[2] - last[2])
        last = g
        for k, v in errors().items():
            max_err[k] = max(max_err[k], abs(v))

    def pause_and_sample() -> None:
        n.spin_for(0.7)  # dur, tüm kaynaklar güncellensin (gecikmeden arınmış örnek)
        for k, v in errors().items():
            still_err[k] = max(still_err[k], abs(v))

    def go(tx: float, ty: float, tyaw: Optional[float], tol: float = 0.05,
           timeout: float = 90.0) -> None:
        t0 = n.sim_t
        while not n.control_step(ekf_rel, tx, ty, tyaw, tol):
            rclpy.spin_once(n, timeout_sec=0.02)
            track()
            if n.sim_t - t0 > timeout:
                raise RuntimeError(f'hedefe ulaşılamadı ({tx:.2f}, {ty:.2f})')

    next_log = 60.0
    while n.sim_t - sim0 < duration:
        tx, ty = rng.uniform(-box_x, box_x), rng.uniform(-box_y, box_y)
        go(tx, ty, None)
        targets += 1
        pause_and_sample()
        if rng.random() < 0.3:  # yerinde dönüş (yaw stresi)
            spins += 1
            y0 = n.ekf[2]
            ang = rng.uniform(math.pi, 2 * math.pi) * rng.choice((-1, 1))
            w = math.copysign(0.8, ang)
            while abs(n.ekf[2] - y0) < abs(ang):
                n.send(0.0, w)
                rclpy.spin_once(n, timeout_sec=0.02)
                track()
            pause_and_sample()
        if n.sim_t - sim0 >= next_log:
            log.info(f'  t={n.sim_t - sim0:5.0f} s  hedef={targets}  yol={path:6.2f} m  '
                     f'dönüş={math.degrees(rot):7.0f}°')
            next_log += 60.0

    drive_time = n.sim_t - sim0
    go(0.0, 0.0, 0.0, tol=0.01)  # EKF tahminine göre başlangıç pozu
    n.spin_for(2.0)
    track()

    rg = relative(g0, n.gt)        # gerçek: başlangıca göre nerede?
    re = relative(e0, n.ekf)       # EKF: başlangıca göre nerede sanıyor?
    fin = errors()
    log.info('=' * 72)
    log.info(f'DRIFT TESTİ  seed={seed}  sürüş={drive_time:.0f} sim-s  '
             f'(+dönüş, toplam {n.sim_t - sim0:.0f} s)  '
             f'RTF={(n.sim_t - sim0) / (time.monotonic() - wall0):.3f}')
    log.info(f'  yol (gerçek)={path:.2f} m  toplam dönüş={math.degrees(rot):.0f}°  '
             f'hedef={targets}  yerinde dönüş={spins}')
    log.info(f'  Başlangıca dönüş: EKF kendini Δxy={math.hypot(re[0], re[1]) * 100:.2f} cm '
             f'Δyaw={math.degrees(wrap(re[2])):+.2f}° uzakta sanıyor; gerçekte '
             f'Δxy={math.hypot(rg[0], rg[1]) * 100:.2f} cm  '
             f'Δyaw={math.degrees(wrap(rg[2])):+.2f}°')

    def fmt(e: dict, key: str) -> str:
        return f'xy={e[key + "_xy"] * 100:5.2f} cm  yaw={math.degrees(e[key + "_yaw"]):+6.2f}°'

    log.info(f'  DRIFT (son, tahmin−gerçek) EKF: {fmt(fin, "ekf")}   '
             f'tekerlek odom: {fmt(fin, "odom")}')
    log.info(f'  Maks. hata, duraklarken    EKF: {fmt(still_err, "ekf")}   '
             f'tekerlek odom: {fmt(still_err, "odom")}')
    log.info(f'  Maks. hata, hareket halinde (yayın gecikmesi dahil) EKF: {fmt(max_err, "ekf")}   '
             f'tekerlek odom: {fmt(max_err, "odom")}')
    ok = fin['ekf_xy'] < 0.10 and abs(math.degrees(fin['ekf_yaw'])) < 5.0
    log.info(f'  Hedef EKF drift (< 10 cm ve < 5°): {"GEÇTİ" if ok else "KALDI"}')
    log.info('=' * 72)


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('basic', 'drift'))
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--duration', type=float, default=300.0, help='drift: sürüş süresi (sim-s)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--box-x', type=float, default=3.5,
                    help='drift: hedeflerin başlangıca göre ±x sınırı (m)')
    ap.add_argument('--box-y', type=float, default=1.5,
                    help='drift: hedeflerin başlangıca göre ±y sınırı (m)')
    args = ap.parse_args(argv)

    rclpy.init()
    node = DriveTester(args.world, args.model, args.cmd_topic)
    try:
        if args.mode == 'basic':
            run_basic(node)
        else:
            run_drift(node, args.duration, args.seed, args.box_x, args.box_y)
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
