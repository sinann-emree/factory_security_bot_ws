#!/usr/bin/env python3
"""
teleop_twist_keyboard'u sanal terminalde (pty) çalıştırıp tuş gönderir.

Hareket, Gazebo gerçek konumuyla ölçülür.

TwistStamped (stamped:=true) teleop zincirinin uçtan uca çalıştığını doğrular.
Kullanım (simülasyon çalışırken):
  ros2 run fsb_gazebo teleop_test.py
"""

import argparse
import math
import os
import pty
import select
import sys
import threading
import time
from typing import Dict, Tuple

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--turn', type=float, default=0.8)
    args = ap.parse_args(argv)

    # Önce fork (rclpy init'ten önce), çocuk süreç teleop'u gerçek bir tty'de çalıştırır.
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp('ros2', [
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-args',
            '-p', 'stamped:=true', '-p', 'frame_id:=base_footprint',
            '-p', f'speed:={args.speed}', '-p', f'turn:={args.turn}',
            '-p', 'use_sim_time:=true', '-r', f'cmd_vel:={args.cmd_topic}'])

    rclpy.init()
    log = rclpy.logging.get_logger('fsb_teleop_test')
    gt: Dict[str, Tuple[float, float, float]] = {}
    lock = threading.Lock()

    def on_pose(m: Pose_V) -> None:
        for p in m.pose:
            if p.name == args.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with lock:
                    gt['p'] = (p.position.x, p.position.y, yaw)

    gz = GzNode()
    gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', on_pose)

    def drain() -> None:
        while select.select([fd], [], [], 0)[0]:
            try:
                os.read(fd, 4096)
            except OSError:
                return

    def hold(key: str, seconds: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            os.write(fd, key.encode())
            time.sleep(0.1)  # diff_drive cmd_vel_timeout (0.5 s) altında tekrar
            drain()

    def pose() -> Tuple[float, float, float]:
        with lock:
            return gt['p']

    try:
        time.sleep(5.0)
        drain()
        t0 = time.monotonic()
        while 'p' not in gt:
            if time.monotonic() - t0 > 20.0:
                log.error('Gazebo gerçek konum verisi yok')
                return 1
            time.sleep(0.1)
        cases = (('i', 4.0, 'ileri (i)', args.speed * 4.0, 0.0),
                 ('j', 3.0, 'sola dön (j)', 0.0, math.degrees(args.turn * 3.0)),
                 (',', 2.0, 'geri (,)', -args.speed * 2.0, 0.0))
        for key, sec, label, exp_d, exp_yaw in cases:
            a = pose()
            hold(key, sec)
            hold('k', 1.0)  # dur
            b = pose()
            dx, dy = b[0] - a[0], b[1] - a[1]
            fwd = dx * math.cos(a[2]) + dy * math.sin(a[2])
            dyaw = math.degrees(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
            log.info(f'teleop {label:13s} {sec:.0f} s: ileri bileşen={fwd:+.3f} m '
                     f'(beklenen {exp_d:+.3f})  Δyaw={dyaw:+.1f}° (beklenen {exp_yaw:+.1f}°)')
    finally:
        os.write(fd, b'\x03')
        time.sleep(1.0)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
