#!/usr/bin/env python3
"""
SLAM haritalaması için factory.sdf dünyasında önceden tanımlı yavaş tur.

Robot EKF tahminine (/odometry/filtered) göre noktadan noktaya sürülür (Nav2 gerekmez).
Robotun (0, 0, 0)'da spawn edildiği ve EKF'nin oradan başladığı varsayılır (odom = dünya).
Hız SLAM için düşük: 0.15 m/s, dönüş ≤ 0.4 rad/s (hızlı dönüş SLAM haritasını bozar).

Rota (dünya koordinatı, engellerden ≥ 0.3 m boşluk):
  kuzey turu: orta koridor → batı ucu → kuzey dış koridor → doğu ucu → kuzey ara koridor → orta
  güney turu: güney ara koridor (batı) → batı → güney dış koridor → doğu → güney ara koridor (doğu)
  → başlangıç (döngü kapatma için orta koridordan geri)
Kullanım (localization:=slam ile simülasyon çalışırken):
  ros2 run fsb_gazebo mapping_tour.py
"""

import math
import sys
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

ROUTE: List[Tuple[float, float]] = [
    # kuzey turu
    (-9.0, 0.0), (-9.0, 6.45), (9.0, 6.45), (9.0, 4.25), (3.6, 4.25), (3.6, 0.0), (0.0, 0.0),
    # güney turu
    (-3.6, 0.0), (-3.6, -4.25), (-9.0, -4.25), (-9.0, -6.45), (9.0, -6.45), (9.0, -4.25),
    (3.6, -4.25), (3.6, 0.0),
    # orta koridordan batıya ve geri (başlangıç bölgesini yeniden görerek döngü kapatma)
    (-6.0, 0.0), (0.0, 0.0),
]

V_MAX = 0.15
W_MAX = 0.4


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Tour(Node):

    def __init__(self) -> None:
        super().__init__('fsb_mapping_tour',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose: Optional[Tuple[float, float, float]] = None
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 20)

    def _on_odom(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def send(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)

    def go(self, tx: float, ty: float, timeout: float = 400.0) -> None:
        t0 = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.pose
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                break
            e = wrap(math.atan2(dy, dx) - yaw)
            w = max(-W_MAX, min(W_MAX, 1.5 * e))
            v = 0.0 if abs(e) > 0.2 else max(0.03, min(V_MAX, 0.5 * dist))
            self.send(v, w)
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'hedefe ulaşılamadı ({tx}, {ty})')
        self.send(0.0, 0.0)


def main() -> int:
    rclpy.init()
    n = Tour()
    log = n.get_logger()
    try:
        while n.pose is None:
            rclpy.spin_once(n, timeout_sec=0.1)
        t0 = time.monotonic()
        for i, (x, y) in enumerate(ROUTE, 1):
            n.go(x, y)
            log.info(f'[{i:2d}/{len(ROUTE)}] ({x:+.2f}, {y:+.2f}) ulaşıldı  '
                     f'(geçen {time.monotonic() - t0:.0f} s)')
        for _ in range(50):  # dur ve SLAM son taramaları işlesin
            n.send(0.0, 0.0)
            rclpy.spin_once(n, timeout_sec=0.05)
        log.info('Tur tamamlandı.')
    finally:
        n.send(0.0, 0.0)
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
