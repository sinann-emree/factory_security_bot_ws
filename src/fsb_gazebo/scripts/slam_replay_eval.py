#!/usr/bin/env python3
"""
SLAM çevrimdışı değerlendirme.

Bag oynatılırken SLAM'in map→base_footprint tahminini /gt_pose
(gt_pose_publisher.py ile kaydedilmiş gerçek konum) ile karşılaştırır.

Varsayım: map = dünya çerçevesi (robot (0,0,0)'da spawn, SLAM oradan başlar).
/gt_pose örnekleri 0.5 s geciktirilerek işlenir (SLAM TF'inin yetişmesi için). 8 duvar-s yeni örnek
gelmezse (bag bitti) özet basar: xy hatası ort/p95/maks (cm), yaw hatası ort/p95/maks (°).

Tipik akış (SLAM parametresi denemek için; Gazebo GEREKMEZ):
  ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file <yaml> \
      -r __node:=slam_toolbox -p use_sim_time:=true &
  ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
  ros2 run fsb_gazebo slam_replay_eval.py --ros-args -p use_sim_time:=true &
  ros2 bag play tour_bag --topics /scan /tf /tf_static /clock /gt_pose
  ros2 run nav2_map_server map_saver_cli -f /tmp/map --ros-args -p use_sim_time:=true
  ros2 run fsb_gazebo map_quality_check.py --map /tmp/map.yaml
Not: bag'deki /tf'te map→odom OLMAMALI (kayıt SLAM/AMCL kapalıyken yapılmalı).
"""

from collections import deque
import math
import sys
import time
from typing import Deque, List, Optional

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_s(m: Odometry) -> float:
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class SlamReplayEval(Node):
    def __init__(self) -> None:
        super().__init__('fsb_slam_replay_eval')
        self.buf = Buffer(cache_time=Duration(seconds=60))
        self.tfl = TransformListener(self.buf, self)
        self.exy: List[float] = []
        self.eyaw: List[float] = []
        self.pend: Deque[Odometry] = deque()
        self.last_log: Optional[int] = None
        self.create_subscription(Odometry, '/gt_pose', self._on_gt_raw, 50)

    def _on_gt_raw(self, m: Odometry) -> None:
        self.pend.append(m)
        now = stamp_s(m)
        while self.pend and now - stamp_s(self.pend[0]) > 0.5:
            self._evaluate(self.pend.popleft())

    def _evaluate(self, m: Odometry) -> None:
        try:
            mo = self.buf.lookup_transform('map', 'odom', Time()).transform
            ob = self.buf.lookup_transform('odom', 'base_footprint',
                                           Time.from_msg(m.header.stamp)).transform
        except Exception:  # noqa: BLE001 - TF henüz yok / aralık dışı
            return
        a = yaw_of(mo.rotation)
        c, s = math.cos(a), math.sin(a)
        x = mo.translation.x + c * ob.translation.x - s * ob.translation.y
        y = mo.translation.y + s * ob.translation.x + c * ob.translation.y
        gp = m.pose.pose
        e = math.hypot(x - gp.position.x, y - gp.position.y)
        ea = abs(math.degrees(wrap(a + yaw_of(ob.rotation) - yaw_of(gp.orientation))))
        self.exy.append(e)
        self.eyaw.append(ea)
        ts = m.header.stamp.sec
        if self.last_log is None or ts - self.last_log >= 60:
            self.last_log = ts
            self.get_logger().info(
                f't={ts:5d} gt=({gp.position.x:+6.2f},{gp.position.y:+6.2f}) '
                f'hata={e * 100:6.1f} cm {ea:5.2f}°')

    def summary(self) -> None:
        if not self.exy:
            self.get_logger().error('hiç örnek değerlendirilemedi (TF / /gt_pose yok?)')
            return
        a, b = np.array(self.exy), np.array(self.eyaw)
        self.get_logger().info(
            f'ÖZET n={len(a)} xy ort={a.mean() * 100:.1f} p95={np.percentile(a, 95) * 100:.1f} '
            f'maks={a.max() * 100:.1f} cm | yaw ort={b.mean():.2f} '
            f'p95={np.percentile(b, 95):.2f} maks={b.max():.2f}°')


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = SlamReplayEval()
    idle = time.monotonic()
    cnt = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if len(node.exy) != cnt:
                cnt = len(node.exy)
                idle = time.monotonic()
            if cnt > 100 and time.monotonic() - idle > 8:
                break
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
