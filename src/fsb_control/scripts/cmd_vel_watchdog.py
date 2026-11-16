#!/usr/bin/env python3
"""
cmd_vel_watchdog: hız komutu kesilirse robotu durduran röle (robot tarafında çalışır).

  cmd_vel_in (TwistStamped) ──▶ [watchdog] ──▶ cmd_vel_out (TwistStamped)

Gelen her komut aynen iletilir. Son komuttan beri `timeout` (varsayılan 0.5 s) geçerse sıfır hız
yayınlanır ve komut yeniden gelene dek `zero_rate` Hz ile yinelenir. Navigasyon / algı başka bir
makinede (sunucu) çalışırken ağ koparsa ya da sunucu çökerse robotun son komutla yürümeye devam
etmesini önler. diff_drive_controller'ın kendi `cmd_vel_timeout`'una ek, ondan bağımsız bir
katmandır (controller parametresi yanlışlıkla değişse de çalışır).

Zaman aşımı düğümün kendi saatine (komutun ALINDIĞI an) göre ölçülür, mesaj damgasına göre değil:
sunucu ile robot saatleri ayrışsa da doğru çalışır. use_sim_time:=true iken sim zamanı kullanılır.

Parametreler: timeout (0.5 s), zero_rate (10.0 Hz; sıfır komut yineleme), check_rate (50.0 Hz;
              zaman aşımı denetimi → en geç timeout + 1/check_rate içinde durur),
              frame_id (base_footprint; sıfır komutun çerçevesi)
Durum: ~/timed_out (std_msgs/Bool, değişince; transient-local)

Kullanım:
  ros2 run fsb_control cmd_vel_watchdog.py --ros-args -r cmd_vel_in:=/cmd_vel \
      -r cmd_vel_out:=/cmd_vel_safe -p use_sim_time:=true
"""

from typing import List, Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelWatchdog(Node):

    def __init__(self) -> None:
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('zero_rate', 10.0)
        self.declare_parameter('check_rate', 50.0)
        self.declare_parameter('frame_id', 'base_footprint')
        self.timeout = float(self.get_parameter('timeout').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.last_rx: Optional[float] = None   # son komutun alındığı an (düğüm saati, s)
        self.timed_out = False
        self.stops = 0
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.state_pub = self.create_publisher(
            Bool, '~/timed_out', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TwistStamped, 'cmd_vel_in', self._on_cmd, 10)
        self.zero_period = 1.0 / max(float(self.get_parameter('zero_rate').value), 1.0)
        self.last_zero = -1e9
        check = max(float(self.get_parameter('check_rate').value), 1.0 / self.zero_period)
        self.create_timer(1.0 / check, self._tick)
        self.state_pub.publish(Bool(data=False))
        self.get_logger().info(f'cmd_vel watchdog: {self.timeout:.2f} s komut gelmezse dur')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: TwistStamped) -> None:
        self.last_rx = self.now()
        if self.timed_out:
            self.timed_out = False
            self.state_pub.publish(Bool(data=False))
            self.get_logger().info('Komut yeniden geliyor; röle açık')
        self.pub.publish(msg)

    def _tick(self) -> None:
        if self.last_rx is None:
            return   # henüz hiç komut gelmedi: yayın yok (controller zaten duruyor)
        now = self.now()
        if now - self.last_rx < self.timeout:
            return
        if not self.timed_out:
            self.timed_out = True
            self.stops += 1
            self.state_pub.publish(Bool(data=True))
            self.get_logger().warning(
                f'{self.timeout:.2f} s komut gelmedi: SIFIR HIZ yayınlanıyor (#{self.stops})')
        elif now - self.last_zero < self.zero_period:
            return
        self.last_zero = now
        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = self.frame_id
        self.pub.publish(zero)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
