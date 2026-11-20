#!/usr/bin/env python3
"""
Gazebo gerçek konumunu /gt_pose (nav_msgs/Odometry, sim zamanı, frame 'world') olarak yayınlar.

YALNIZCA rosbag kaydı için (SLAM parametrelerini çevrimdışı değerlendirmek,
bkz. slam_replay_eval.py).
Navigasyon yığını bu topic'i kullanmaz; köprüye ground truth eklenmez. Gerçek konum doğrudan
gz-transport `/world/<world>/dynamic_pose/info`'dan okunur.

Kullanım:
  ros2 run fsb_gazebo gt_pose_publisher.py [--world factory --model fsb_robot --rate 20] \
      --ros-args -p use_sim_time:=true
  ros2 bag record /scan /tf /tf_static /clock /gt_pose -o tour_bag
"""

import argparse
import sys
from typing import Dict

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import rclpy.utilities


class GtPosePublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('fsb_gt_pose_publisher')
        self.model = args.model
        self.pub = self.create_publisher(Odometry, '/gt_pose', 50)
        self.last: Dict[str, object] = {}
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', self._on_gt)
        self.create_timer(1.0 / args.rate, self._tick)

    def _on_gt(self, msg: Pose_V) -> None:
        for ps in msg.pose:
            if ps.name == self.model:
                self.last['p'] = ps

    def _tick(self) -> None:
        ps = self.last.get('p')
        if ps is None:
            return
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = ps.position.x, ps.position.y
        q = o.pose.pose.orientation
        q.x, q.y, q.z, q.w = ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w
        self.pub.publish(o)


def main(argv=None) -> int:
    argv_all = argv if argv is not None else sys.argv
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--rate', type=float, default=20.0, help='Hz (sim saati)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv_all)[1:])
    rclpy.init(args=argv_all)
    node = GtPosePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
