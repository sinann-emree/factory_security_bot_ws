#!/usr/bin/env python3
"""
Dinamik engel testi (Aşama 5 çıkış kriteri).

1) NavigateToPose ile robot A=(-4.5, 0, 0°)'a götürülür.
2) B=(4.5, 0, 0°) hedefi gönderilir. Robot x > --trigger-x geçince, rotası üzerine
   (haritada olmayan) bir kutu Gazebo'da spawn edilir (/world/<world>/create).
   Kutu dinamiktir (kütleli): çarpışma olursa yerinden oynar → kutu yer değiştirmesi
   çarpışma kanıtıdır.
3) Ölçülen: hedef sonucu, robot ayak izi (footprint) ile kutu arasındaki minimum açıklık,
   en büyük yanal sapma, kutu yer değiştirmesi, süre. Sonda kutu silinir (/world/<world>/remove).
Kullanım (bringup localization:=amcl çalışırken, devriye kapalıyken):
  ros2 run fsb_gazebo dynamic_obstacle_test.py
"""

import argparse
import math
import sys
import time
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_msgs.action import NavigateToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

Pose2D = Tuple[float, float, float]

# Ayak izi: fsb_navigation/config/nav2_params.yaml footprint ile aynı (fsb_properties.xacro'dan)
FOOTPRINT = np.array([[0.070, 0.087], [0.070, -0.087], [-0.204, -0.087], [-0.204, 0.087]])

BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {h2} 0 0 0</pose>
    <link name="link">
      <inertial><mass>5.0</mass>
        <inertia><ixx>0.2083</ixx><iyy>0.2083</iyy><izz>0.2083</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>{s} {s} {h}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{s} {s} {h}</size></box></geometry>
        <material><diffuse>0.9 0.1 0.1 1</diffuse><ambient>0.9 0.1 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>"""


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def footprint_points(p: Pose2D, step: float = 0.01) -> np.ndarray:
    pts = []
    for i in range(len(FOOTPRINT)):
        a, b = FOOTPRINT[i], FOOTPRINT[(i + 1) % len(FOOTPRINT)]
        k = max(2, int(np.linalg.norm(b - a) / step))
        pts.append(a + np.linspace(0, 1, k)[:, None] * (b - a))
    pts = np.vstack(pts)
    c, s = math.cos(p[2]), math.sin(p[2])
    return pts @ np.array([[c, s], [-s, c]]) + np.array([p[0], p[1]])


def box_clearance(pts: np.ndarray, bx: float, by: float, half: float) -> float:
    dx = np.abs(pts[:, 0] - bx) - half
    dy = np.abs(pts[:, 1] - by) - half
    sd = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0)
    return float(sd.min())


class ObstacleTest(Node):

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('fsb_dynamic_obstacle_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.gt: Optional[Pose2D] = None
        self.box: Optional[Tuple[float, float]] = None
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', self._on_gt):
            raise RuntimeError('gz-transport aboneliği başarısız')

    def _on_gt(self, m: Pose_V) -> None:
        for ps in m.pose:
            if ps.name == self.args.model:
                self.gt = (ps.position.x, ps.position.y, yaw_of(ps.orientation))
            elif ps.name == self.args.box_name:
                self.box = (ps.position.x, ps.position.y)

    def goal(self, x: float, y: float, yaw: float):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        if not h.accepted:
            raise RuntimeError('hedef reddedildi')
        return h, h.get_result_async()

    def spawn_box(self) -> bool:
        a = self.args
        req = EntityFactory()
        req.sdf = BOX_SDF.format(name=a.box_name, x=a.box_x, y=a.box_y, s=a.box_size,
                                 h=a.box_size, h2=a.box_size / 2.0)
        ok, rep = self._gz.request(f'/world/{a.world}/create', req, EntityFactory, Boolean, 3000)
        return bool(ok and rep.data)

    def remove_box(self) -> bool:
        req = Entity()
        req.name = self.args.box_name
        req.type = Entity.MODEL
        ok, rep = self._gz.request(f'/world/{self.args.world}/remove', req, Entity, Boolean, 3000)
        return bool(ok and rep.data)


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--start', type=float, nargs=2, default=[-4.5, 0.0])
    ap.add_argument('--goal', type=float, nargs=2, default=[4.5, 0.0])
    ap.add_argument('--trigger-x', type=float, default=-2.5)
    ap.add_argument('--box-x', type=float, default=0.5)
    ap.add_argument('--box-y', type=float, default=0.0)
    ap.add_argument('--box-size', type=float, default=0.5)
    ap.add_argument('--box-name', default='fsb_dyn_obstacle')
    args = ap.parse_args(argv)

    rclpy.init()
    n = ObstacleTest(args)
    log = n.get_logger()
    spawned = False
    try:
        while not n.nav.wait_for_server(timeout_sec=2.0):
            log.info('navigate_to_pose bekleniyor...')
        while n.gt is None:
            rclpy.spin_once(n, timeout_sec=0.1)

        log.info(f'1) Başlangıca gidiliyor {tuple(args.start)}')
        _, res = n.goal(args.start[0], args.start[1], 0.0)
        rclpy.spin_until_future_complete(n, res)
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'başlangıca gidilemedi (durum {res.result().status})')

        log.info(f'2) Hedef {tuple(args.goal)}; x > {args.trigger_x} olunca kutu '
                 f'({args.box_x}, {args.box_y}) {args.box_size} m spawn edilecek')
        _, res = n.goal(args.goal[0], args.goal[1], 0.0)
        t0 = time.monotonic()
        min_clear = math.inf
        max_dev = 0.0
        box0: Optional[Tuple[float, float]] = None
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            g = n.gt
            max_dev = max(max_dev, abs(g[1] - args.box_y)) if spawned else max_dev
            if not spawned and g[0] > args.trigger_x:
                spawned = n.spawn_box()
                log.info(f'   kutu spawn: {"OK" if spawned else "BAŞARISIZ"} '
                         f'(robot x={g[0]:.2f}, kutuya {args.box_x - g[0]:.2f} m)')
                if not spawned:
                    raise RuntimeError('kutu spawn edilemedi')
            if spawned and n.box is not None:
                if box0 is None:
                    box0 = n.box
                min_clear = min(min_clear, box_clearance(
                    footprint_points(g), n.box[0], n.box[1], args.box_size / 2.0))
            if time.monotonic() - t0 > 300.0:
                raise RuntimeError('zaman aşımı')
        status = res.result().status
        g = n.gt
        rclpy.spin_once(n, timeout_sec=0.2)
        moved = math.hypot(n.box[0] - box0[0], n.box[1] - box0[1]) if box0 and n.box else math.nan
        goal_err = math.hypot(g[0] - args.goal[0], g[1] - args.goal[1])
        ok = status == GoalStatus.STATUS_SUCCEEDED and min_clear > 0.0 and moved < 0.01
        log.info('=' * 70)
        st = 'SUCCEEDED' if status == GoalStatus.STATUS_SUCCEEDED else str(status)
        log.info(f'Hedef durumu: {st}  hedefe uzaklık (gerçek)={goal_err * 100:.1f} cm  '
                 f'süre={time.monotonic() - t0:.1f} s')
        log.info(f'Kutuya min açıklık (ayak izi kenarı → kutu yüzeyi): {min_clear * 100:.1f} cm')
        log.info(f'Maks. yanal sapma (|y - y_kutu|): {max_dev:.2f} m')
        log.info(f'Kutu yer değiştirmesi (çarpışma göstergesi): {moved * 100:.2f} cm')
        log.info(f'SONUÇ: {"GEÇTİ" if ok else "KALDI"}')
        return 0 if ok else 1
    finally:
        if spawned:
            log.info(f'Kutu silindi: {n.remove_box()}')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
