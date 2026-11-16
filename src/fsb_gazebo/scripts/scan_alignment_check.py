#!/usr/bin/env python3
"""
LaserScan ↔ dünya geometrisi hizalama kontrolü (LiDAR 180° montajı doğru mu?).

Her /scan noktası: Gazebo gerçek pozu × TF(base_footprint→scan.frame_id) ile dünyaya yansıtılır,
dünya SDF'indeki statik kutuların (duvar, raf, kolon, palet) en yakın yüzeyine uzaklığı ölçülür.
Karşılaştırma: aynı hesap LiDAR yaw'ı +180° yanlış varsayılarak da yapılır.
Kullanım (simülasyon çalışırken, robot dururken):
  ros2 run fsb_gazebo scan_alignment_check.py
"""

import argparse
import math
import os
import sys
import threading
import time
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

Rect = Tuple[str, float, float, float, float]  # ad, cx, cy, hx, hy


def load_boxes(sdf_path: str) -> List[Rect]:
    rects: List[Rect] = []
    for model in ET.parse(sdf_path).getroot().iter('model'):
        size = model.find('.//collision/geometry/box/size')
        pose = model.find('pose')
        if size is None or pose is None:
            continue
        x, y, _z, _r, _p, yaw = (float(v) for v in pose.text.split())
        if abs(yaw) > 1e-9:
            raise ValueError(f'{model.get("name")}: döndürülmüş kutu desteklenmiyor')
        sx, sy, _sz = (float(v) for v in size.text.split())
        rects.append((model.get('name'), x, y, sx / 2.0, sy / 2.0))
    return rects


def dist_to_boundary(px: float, py: float, r: Rect) -> float:
    _, cx, cy, hx, hy = r
    dx, dy = abs(px - cx) - hx, abs(py - cy) - hy
    return abs(math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0))


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Checker(Node):

    def __init__(self, world: str, model: str) -> None:
        super().__init__('fsb_scan_alignment_check',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scan: Optional[LaserScan] = None
        self.gt: Optional[Tuple[float, float, float]] = None
        self._lock = threading.Lock()
        self._model = model
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self._gz = GzNode()
        self._gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self._on_gt)

    def _on_scan(self, m: LaserScan) -> None:
        self.scan = m

    def _on_gt(self, m: Pose_V) -> None:
        for p in m.pose:
            if p.name == self._model:
                with self._lock:
                    self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))


def evaluate(scan: LaserScan, robot: Tuple[float, float, float],
             mount: Tuple[float, float, float], rects: List[Rect],
             extra_yaw: float, tol: float) -> Tuple[int, float, float]:
    rx, ry, ryaw = robot
    mx, my, myaw = mount
    lx = rx + mx * math.cos(ryaw) - my * math.sin(ryaw)
    ly = ry + mx * math.sin(ryaw) + my * math.cos(ryaw)
    base = ryaw + myaw + extra_yaw
    d: List[float] = []
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = base + scan.angle_min + i * scan.angle_increment
        px, py = lx + r * math.cos(a), ly + r * math.sin(a)
        d.append(min(dist_to_boundary(px, py, rc) for rc in rects))
    if not d:
        return 0, 0.0, float('nan')
    inl = sum(1 for v in d if v < tol)
    d.sort()
    return len(d), 100.0 * inl / len(d), d[len(d) // 2]


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--sdf', default=os.path.join(
        get_package_share_directory('fsb_gazebo'), 'worlds', 'factory.sdf'))
    ap.add_argument('--tol', type=float, default=0.05, help='inlier eşiği (m)')
    args = ap.parse_args(argv)

    rects = load_boxes(args.sdf)
    rclpy.init()
    n = Checker(args.world, args.model)
    log = n.get_logger()
    try:
        t0 = time.monotonic()
        tf = None
        while True:
            rclpy.spin_once(n, timeout_sec=0.1)
            if n.scan is not None and n.gt is not None:
                try:
                    tf = n.tf_buffer.lookup_transform('base_footprint', n.scan.header.frame_id,
                                                      Time())
                    break
                except Exception:  # noqa: BLE001 — TF henüz gelmedi
                    pass
            if time.monotonic() - t0 > 20.0:
                log.error(f'veri yok: scan={n.scan is not None} gt={n.gt is not None} tf={tf}')
                return 1
        scan = n.scan
        with n._lock:
            robot = n.gt
        t = tf.transform
        mount = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        log.info(f'scan frame_id={scan.header.frame_id}  ışın={len(scan.ranges)}  '
                 f'TF base_footprint→{scan.header.frame_id}: xy=({mount[0]:.4f}, {mount[1]:.4f}) '
                 f'yaw={math.degrees(mount[2]):.1f}°')
        log.info(f'robot gerçek poz: ({robot[0]:.3f}, {robot[1]:.3f}) '
                 f'yaw={math.degrees(robot[2]):.1f}°  '
                 f'kutu sayısı={len(rects)}  inlier eşiği={args.tol * 100:.0f} cm')
        for label, extra in (('URDF montajı (doğru olmalı)', 0.0),
                             ('+180° yanlış montaj varsayımı', math.pi)):
            cnt, inl, med = evaluate(scan, robot, mount, rects, extra, args.tol)
            log.info(f'  {label:32s}: geçerli nokta={cnt}  inlier=%{inl:.1f}  '
                     f'medyan uzaklık={med * 100:.2f} cm')
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
