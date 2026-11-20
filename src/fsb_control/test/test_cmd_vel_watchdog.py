"""
cmd_vel_watchdog davranış testleri (gerçek rclpy düğümleri, duvar saati).

Doğrulananlar: ilk komuttan önce yayın yok; komutlar aynen iletilir; komut kesilince `timeout`
sonra sıfır hız yayınlanır ve yinelenir, `~/timed_out` true olur; komut yeniden gelince röle
açılır. Düğüm ayrı bir namespace'te çalışır, başka bir ROS grafiğiyle karışmaz.
"""

import importlib.util
import os
import time
from typing import List, Tuple

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

NS = '/fsb_watchdog_test'
TIMEOUT = 0.3
ZERO_RATE = 10.0
SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts',
                      'cmd_vel_watchdog.py')


def _load_watchdog_module():
    spec = importlib.util.spec_from_file_location('cmd_vel_watchdog', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Probe(Node):

    def __init__(self) -> None:
        super().__init__('watchdog_probe')
        self.pub = self.create_publisher(TwistStamped, f'{NS}/cmd_vel_in', 10)
        self.out: List[Tuple[float, float]] = []      # (alınma anı, linear.x)
        self.state: List[bool] = []
        self.create_subscription(TwistStamped, f'{NS}/cmd_vel_out',
                                 lambda m: self.out.append((time.monotonic(), m.twist.linear.x)),
                                 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, f'{NS}/cmd_vel_watchdog/timed_out',
                                 lambda m: self.state.append(m.data), qos)

    def send(self, vx: float) -> float:
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx
        self.pub.publish(m)
        return time.monotonic()


@pytest.fixture
def graph():
    ctx_args = ['--ros-args',
                '-r', f'cmd_vel_watchdog:__ns:={NS}',
                # Genel parametre: probe bu adları tanımlamadığı için yalnızca watchdog'u etkiler
                '-p', f'timeout:={TIMEOUT}',
                '-p', f'zero_rate:={ZERO_RATE}']
    rclpy.init(args=ctx_args)
    wd = _load_watchdog_module().CmdVelWatchdog()
    probe = Probe()
    assert wd.timeout == pytest.approx(TIMEOUT), 'timeout parametresi uygulanmadı'
    ex = SingleThreadedExecutor()
    ex.add_node(wd)
    ex.add_node(probe)

    def spin_for(sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.01)

    spin_for(0.5)   # keşif (discovery)
    yield probe, spin_for
    ex.shutdown()
    wd.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def _drive(probe: Probe, spin_for, vx: float, duration: float, rate: float = 20.0) -> float:
    last = 0.0
    end = time.monotonic() + duration
    while time.monotonic() < end:
        last = probe.send(vx)
        spin_for(1.0 / rate)
    return last


def test_silent_before_first_command(graph):
    probe, spin_for = graph
    spin_for(2 * TIMEOUT)
    assert probe.out == []
    assert probe.state and probe.state[-1] is False


def test_forwards_commands_unchanged(graph):
    probe, spin_for = graph
    _drive(probe, spin_for, 0.15, 0.5)
    spin_for(0.05)
    assert len(probe.out) >= 5
    assert all(v == pytest.approx(0.15) for _, v in probe.out)
    assert probe.state[-1] is False


def test_zero_velocity_after_timeout_and_recovery(graph):
    probe, spin_for = graph
    t_last = _drive(probe, spin_for, 0.15, 0.4)
    spin_for(TIMEOUT + 0.7)

    zeros = [t for t, v in probe.out if t > t_last and v == 0.0]
    assert zeros, 'zaman aşımında sıfır hız yayınlanmadı'
    delay = zeros[0] - t_last
    assert TIMEOUT - 0.02 <= delay <= TIMEOUT + 0.2, f'gecikme {delay:.3f} s'
    # Sıfır komut zero_rate ile yinelenir (0.7 s içinde ~7; zamanlama payıyla 3..12)
    assert 3 <= len(zeros) <= 12
    # İlk sıfırdan sonra (komut gelmedikçe) yalnızca sıfır hız çıkar
    assert all(v == 0.0 for t, v in probe.out if t >= zeros[0])
    assert probe.state[-1] is True

    n_before = len(probe.out)
    _drive(probe, spin_for, 0.1, 0.3)
    spin_for(0.05)
    resumed = [v for _, v in probe.out[n_before:] if v != 0.0]
    assert resumed and all(v == pytest.approx(0.1) for v in resumed)
    assert probe.state[-1] is False
