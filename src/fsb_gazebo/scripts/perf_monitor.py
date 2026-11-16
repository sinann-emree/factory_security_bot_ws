#!/usr/bin/env python3
"""
Başarım ölçümü: gerçek zaman çarpanı (RTF) ve süreç gruplarına göre CPU kullanımı.

RTF = /clock ilerlemesi / duvar saati. CPU: psutil ile süreç başına, "bir çekirdeğin yüzdesi"
(%100 = 1 tam çekirdek); gruplar komut satırındaki anahtar kelimelerle eşlenir. GPU (varsa)
nvidia-smi ile örneklenir. Algı açık/kapalı karşılaştırması için aynı senaryoda iki kez koşun.

Kullanım:
  ros2 run fsb_gazebo perf_monitor.py --duration 240 [--json çıktı.json] [--label ad]
"""

import argparse
import json
import shutil
import subprocess
import time
from typing import Dict, List, Optional

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

# grup → komut satırında aranan anahtar kelimeler (ilk eşleşen grup kazanır)
GROUPS = [
    ('yolo', ['yolo_node']),
    ('gazebo', ['gz sim', 'gz-sim', 'ruby']),
    ('nav2', ['controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
              'smoother_server', 'velocity_smoother', 'collision_monitor', 'waypoint_follower',
              'lifecycle_manager', 'map_server', 'amcl', 'component_container', 'nav2']),
    ('ekf', ['ekf_node']),
    ('bridge', ['parameter_bridge']),
    ('patrol', ['patrol_executor', 'coverage_monitor', 'patrol_node']),
    ('rsp', ['robot_state_publisher']),
]


def group_of(cmd: str) -> Optional[str]:
    for name, keys in GROUPS:
        if any(k in cmd for k in keys):
            return name
    return None


def gpu_sample() -> Optional[List[float]]:
    if not shutil.which('nvidia-smi'):
        return None
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True,
                             timeout=5).stdout.strip().splitlines()[0]
        return [float(v) for v in out.split(',')]
    except Exception:  # noqa: BLE001 - GPU ölçümü isteğe bağlı
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=240.0, help='duvar saati, s')
    ap.add_argument('--interval', type=float, default=2.0, help='örnekleme aralığı, s')
    ap.add_argument('--json', default='', help='özetin yazılacağı dosya')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    rclpy.init()
    node = Node('fsb_perf_monitor')
    sim = {'t': None}
    node.create_subscription(
        Clock, '/clock',
        lambda m: sim.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9),
        qos_profile_sensor_data)
    t_wait = time.monotonic()
    while sim['t'] is None and time.monotonic() - t_wait < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if sim['t'] is None:
        node.get_logger().error('/clock yok')
        return 1

    procs: Dict[int, psutil.Process] = {}
    cpu: Dict[str, List[float]] = {g: [] for g, _ in GROUPS}
    total: List[float] = []
    rtf: List[float] = []
    gpu: List[List[float]] = []
    psutil.cpu_percent(None)
    t0, s0 = time.monotonic(), sim['t']
    t_prev, s_prev = t0, s0
    while time.monotonic() - t0 < args.duration and rclpy.ok():
        t_end = time.monotonic() + args.interval
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        rtf.append((sim['t'] - s_prev) / (now - t_prev))
        t_prev, s_prev = now, sim['t']
        acc = {g: 0.0 for g, _ in GROUPS}
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                g = group_of(' '.join(p.info['cmdline'] or []))
                if g is None:
                    continue
                if p.pid not in procs:
                    procs[p.pid] = p
                    p.cpu_percent(None)   # ilk çağrı 0 döner; sayaç başlatılır
                    continue
                acc[g] += procs[p.pid].cpu_percent(None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        for g in acc:
            cpu[g].append(acc[g])
        total.append(psutil.cpu_percent(None))
        gs = gpu_sample()
        if gs:
            gpu.append(gs)

    def stat(v: List[float]) -> Dict[str, float]:
        v = sorted(v[1:] or v)   # ilk örnek (sayaç başlatma) atılır
        n = len(v)
        return {'mean': round(sum(v) / n, 2), 'p95': round(v[min(n - 1, int(0.95 * n))], 2),
                'max': round(v[-1], 2)} if n else {}

    wall = time.monotonic() - t0
    out = {
        'label': args.label, 'wall_s': round(wall, 1), 'sim_s': round(sim['t'] - s0, 1),
        'rtf_mean': round((sim['t'] - s0) / wall, 4), 'rtf': stat(rtf),
        'cpu_cores': psutil.cpu_count(),
        'cpu_system_pct_of_all_cores': stat(total),
        'cpu_pct_of_one_core': {g: stat(v) for g, v in cpu.items() if any(v)},
        'gpu_util_pct': stat([g[0] for g in gpu]) if gpu else None,
        'gpu_mem_mb': stat([g[1] for g in gpu]) if gpu else None,
    }
    text = json.dumps(out, indent=2, ensure_ascii=False)
    node.get_logger().info('\n' + text)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
