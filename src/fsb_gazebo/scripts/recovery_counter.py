#!/usr/bin/env python3
"""
Nav2 kurtarma davranışı sayacı.

behavior_server action'larının (spin, backup, drive_on_heading, wait, assisted_teleop)
`_action/status` topic'lerini dinler; her benzersiz goal_id bir çağrıdır.

Her çağrıda `KURTARMA: <ad>` uyarısı; SIGINT/SIGTERM'de `KURTARMA ÖZETİ` basar.
Not: costmap temizleme (ClearCostmap servisleri) action olmadığı için sayılmaz.

Kullanım (Nav2 çalışırken, ör. devriye testi boyunca):
  ros2 run fsb_gazebo recovery_counter.py --ros-args -p use_sim_time:=true
"""
import signal
from typing import Dict, Set

from action_msgs.msg import GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

BEHAVIORS = ['spin', 'backup', 'drive_on_heading', 'wait', 'assisted_teleop']


class RecoveryCounter(Node):
    def __init__(self) -> None:
        super().__init__('fsb_recovery_counter')
        self.seen: Dict[str, Set[bytes]] = {b: set() for b in BEHAVIORS}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for b in BEHAVIORS:
            self.create_subscription(GoalStatusArray, f'/{b}/_action/status',
                                     lambda m, b=b: self._on(b, m), qos)

    def _on(self, name: str, msg: GoalStatusArray) -> None:
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if gid not in self.seen[name]:
                self.seen[name].add(gid)
                t = st.goal_info.stamp.sec + st.goal_info.stamp.nanosec * 1e-9
                self.get_logger().warn(f'KURTARMA: {name} (sim t={t:.1f})')

    def summary(self) -> None:
        tot = sum(len(v) for v in self.seen.values())
        parts = ', '.join(f'{k}={len(v)}' for k, v in self.seen.items())
        self.get_logger().info(f'KURTARMA ÖZETİ: toplam={tot}  ({parts})')


def main() -> None:
    rclpy.init()
    node = RecoveryCounter()
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('v', True))
    while rclpy.ok() and not stop['v']:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.summary()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
