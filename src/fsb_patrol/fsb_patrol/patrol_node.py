"""
Döngüsel devriye düğümü (Nav2 FollowWaypoints).

Her tur için bir FollowWaypoints hedefi gönderilir (number_of_loops=0); böylece tur
süresi ve kaçırılan waypoint'ler tur bazında loglanır. Bir tur başarısız olursa
(hedef reddi/iptal/kaçırılan nokta) düğüm durmaz, bir sonraki turu dener.

Parametreler:
  waypoints (double[]): düz liste [x1, y1, yaw1, x2, y2, yaw2, ...]  (m, m, rad)
  frame_id (string):    waypoint çerçevesi (varsayılan: map)
  loops (int):          tur sayısı, 0 = sonsuz
  action_name (string): FollowWaypoints action adı (varsayılan: follow_waypoints)
"""

import math
import time
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future


class PatrolNode(Node):
    """FollowWaypoints action istemcisiyle tur tur devriye."""

    def __init__(self) -> None:
        super().__init__('fsb_patrol')
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('loops', 0)
        self.declare_parameter('action_name', 'follow_waypoints')

        flat: List[float] = list(self.get_parameter('waypoints').value)
        if len(flat) % 3 != 0 or len(flat) < 9:
            raise ValueError('waypoints en az 3 nokta içeren [x, y, yaw, ...] listesi olmalı')
        self.points = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.frame_id: str = self.get_parameter('frame_id').value
        self.loops: int = int(self.get_parameter('loops').value)
        self.client = ActionClient(self, FollowWaypoints,
                                   self.get_parameter('action_name').value)

        self.lap = 0
        self.ok_laps = 0
        self.failed_laps = 0
        self.lap_start = 0.0
        self.lap_start_sim = 0.0
        self.done = False
        self._goal_handle = None
        self.get_logger().info(
            f'{len(self.points)} waypoint, frame={self.frame_id}, '
            f'tur={"sonsuz" if self.loops == 0 else self.loops}')

    def _poses(self) -> List[PoseStamped]:
        out = []
        for x, y, yaw in self.points:
            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            out.append(p)
        return out

    def start(self) -> None:
        """Bekle: action sunucusu hazır olunca ilk turu başlat."""
        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('follow_waypoints action sunucusu bekleniyor...')
        self._send_lap()

    def _send_lap(self) -> None:
        if self.loops and self.lap >= self.loops:
            self.get_logger().info(
                f'DEVRİYE BİTTİ: {self.ok_laps}/{self.lap} tur başarılı, '
                f'{self.failed_laps} başarısız')
            self.done = True
            return
        self.lap += 1
        goal = FollowWaypoints.Goal()
        goal.number_of_loops = 0
        goal.poses = self._poses()
        self.lap_start = time.monotonic()
        self.lap_start_sim = self._sim_now()
        self.get_logger().info(f'Tur {self.lap} başladı')
        self.client.send_goal_async(goal, feedback_callback=self._on_feedback) \
            .add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb: FollowWaypoints.Impl.FeedbackMessage) -> None:
        idx = fb.feedback.current_waypoint
        if idx != getattr(self, '_last_idx', None):
            self._last_idx = idx
            self.get_logger().info(f'  Tur {self.lap}: hedef W{idx + 1}')

    def _on_goal_response(self, fut: Future) -> None:
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error(f'Tur {self.lap}: hedef reddedildi')
            self.failed_laps += 1
            self._retry_later()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut: Future) -> None:
        res = fut.result()
        dt = time.monotonic() - self.lap_start
        dts = self._sim_now() - self.lap_start_sim  # use_sim_time'da sim süresi (RTF'den bağımsız)
        missed = [m.index for m in res.result.missed_waypoints]
        self._last_idx = None
        if res.status == GoalStatus.STATUS_SUCCEEDED and not missed:
            self.ok_laps += 1
            self.get_logger().info(
                f'Tur {self.lap} TAMAM (sim {dts:.1f} s / duvar {dt:.1f} s)  '
                f'başarılı={self.ok_laps}/{self.lap}')
        else:
            self.failed_laps += 1
            self.get_logger().warn(
                f'Tur {self.lap} BAŞARISIZ (sim {dts:.1f} s) durum={res.status} '
                f'kaçırılan={[i + 1 for i in missed]} hata={res.result.error_code} '
                f'{res.result.error_msg}')
        self._send_lap()

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _retry_later(self) -> None:
        self._retry_timer: Optional[rclpy.timer.Timer] = None

        def fire() -> None:
            self._retry_timer.cancel()
            self._send_lap()
        self._retry_timer = self.create_timer(2.0, fire)

    def cancel(self) -> None:
        """İptal et: etkin hedef (kapanışta)."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        node.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
