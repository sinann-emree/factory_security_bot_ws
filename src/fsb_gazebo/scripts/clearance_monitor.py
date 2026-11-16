#!/usr/bin/env python3
"""
Çarpışma / açıklık izleyici (simülasyon test aracı, ground truth).

Gazebo gerçek pozu (gz-transport `/world/<world>/dynamic_pose/info`) + robot footprint'i
(nav2_params.yaml `local_costmap.footprint`) ile dünya SDF'indeki tüm kutu engellere (duvar, raf,
kolon, palet) en kısa mesafeyi ölçer. Footprint kenarı 1 cm aralıkla örneklenir; işaretli uzaklık
< 0 = içe girme (çarpışma). Ayrıca kutuların konumu açılışta ve kapanışta karşılaştırılır
(itilen engel = çarpışma göstergesi; statik dünyada değişmemeli).

Rapor: min açıklık, < 2 cm "temas yakını" süreleri, çarpışma (≤ 0) örnek sayısı, açıklık dağılımı.
SIGINT/SIGTERM (arka planda da) veya --duration (sim-s yerine gerçek poz örneği zamanı) ile biter.

Kullanım:
  ros2 run fsb_gazebo clearance_monitor.py [--duration 0] [--report 300]
"""

import argparse
import logging
import math
import os
import signal
import sys
import threading
import time
from typing import List

from ament_index_python.packages import get_package_share_directory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_quality_check import load_boxes  # noqa: E402,I100 (aynı dizindeki araç)


def footprint_from_nav2(path: str) -> np.ndarray:
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    fp = d['local_costmap']['local_costmap']['ros__parameters']['footprint']
    return np.array(yaml.safe_load(fp), dtype=float)


def perimeter_samples(poly: np.ndarray, step: float = 0.01) -> np.ndarray:
    pts: List[np.ndarray] = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        t = np.arange(n)[:, None] / n
        pts.append(a + t * (b - a))
    return np.vstack(pts)


def signed_dist(px: np.ndarray, py: np.ndarray, rects) -> np.ndarray:
    best = np.full(px.shape, np.inf)
    for _, cx, cy, hx, hy in rects:
        dx = np.abs(px - cx) - hx
        dy = np.abs(py - cy) - hy
        sd = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0)
        best = np.minimum(best, sd)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='factory')
    ap.add_argument('--model', default='fsb_robot')
    ap.add_argument('--sdf', default=os.path.join(
        get_package_share_directory('fsb_gazebo'), 'worlds', 'factory.sdf'))
    ap.add_argument('--nav2-params', default=os.path.join(
        get_package_share_directory('fsb_navigation'), 'config', 'nav2_params.yaml'))
    ap.add_argument('--duration', type=float, default=0.0, help='sim-s, 0 = sinyale kadar')
    ap.add_argument('--report', type=float, default=300.0, help='ara özet (sim-s)')
    args = ap.parse_args()

    rects = load_boxes(args.sdf)
    fp = footprint_from_nav2(args.nav2_params)
    samples = perimeter_samples(fp)
    lock = threading.Lock()
    state = {'n': 0, 'min': math.inf, 'min_at': None, 'hit': 0, 'near_t': 0.0, 't0': None,
             't': None, 'last_t': None, 'hist': np.zeros(6, dtype=np.int64), 'stop': False,
             'next': None, 'boxes0': None, 'boxes': None, 'path': 0.0, 'last_xy': None}
    edges = np.array([0.0, 0.02, 0.05, 0.10, 0.20, 0.30])
    box_names = {r[0] for r in rects}

    logging.basicConfig(level=logging.INFO, format='[clearance_monitor] %(message)s')
    logger = logging.getLogger('clearance_monitor')

    def log(msg: str) -> None:
        logger.info(msg)  # gz-transport süreci (ROS düğümü değil) → Python logging

    def summary(tag: str) -> None:
        s = state
        el = (s['t'] - s['t0']) if s['t0'] is not None else 0.0
        h = s['hist']
        tot = max(1, int(h.sum()))
        hi = list(edges[1:] * 100) + [999]
        dist = '  '.join(f'[{edges[i] * 100:.0f},{hi[i]:.0f})cm %{100 * h[i] / tot:.1f}'
                         for i in range(len(edges)))
        mx, my = s['min_at'] if s['min_at'] else (math.nan, math.nan)
        log(f'--- {tag}: {el:.0f} sim-s, {s["n"]} örnek, yol {s["path"]:.1f} m --- '
            f'min açıklık {100 * s["min"]:.1f} cm @ ({mx:+.2f},{my:+.2f}); '
            f'çarpışma (≤0) örnek {s["hit"]}; <2 cm süre {s["near_t"]:.1f} s')
        log(f'    açıklık dağılımı: {dist}')

    def on_pose(msg: Pose_V) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9  # gz.msgs.Time: sec/nsec
        with lock:
            boxes = {}
            robot = None
            for ps in msg.pose:
                if ps.name == args.model:
                    robot = ps
                elif ps.name in box_names:
                    boxes[ps.name] = (ps.position.x, ps.position.y)
            if boxes:
                if state['boxes0'] is None:
                    state['boxes0'] = boxes
                state['boxes'] = boxes
            if robot is None:
                return
            q = robot.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            c, s_ = math.cos(yaw), math.sin(yaw)
            x, y = robot.position.x, robot.position.y
            wx = x + c * samples[:, 0] - s_ * samples[:, 1]
            wy = y + s_ * samples[:, 0] + c * samples[:, 1]
            d = signed_dist(wx, wy, rects)
            k = int(np.argmin(d))
            dmin = float(d[k])
            if state['t0'] is None:
                state['t0'] = t
                state['next'] = t + args.report
            dt = t - state['last_t'] if state['last_t'] is not None else 0.0
            state['last_t'] = state['t'] = t
            state['n'] += 1
            if state['last_xy'] is not None:
                state['path'] += math.hypot(x - state['last_xy'][0], y - state['last_xy'][1])
            state['last_xy'] = (x, y)
            if dmin < state['min']:
                state['min'], state['min_at'] = dmin, (float(wx[k]), float(wy[k]))
            if dmin <= 0.0:
                state['hit'] += 1
            if dmin < 0.02:
                state['near_t'] += dt
            state['hist'][max(0, int(np.searchsorted(edges, dmin, side='right')) - 1)] += 1
            if t >= state['next']:
                state['next'] += args.report
                summary('ARA ÖZET')
            if args.duration and t - state['t0'] >= args.duration:
                state['stop'] = True

    node = GzNode()
    # dynamic_pose yalnızca hareketli modelleri içerir; statik kutuların konumu için pose/info
    node.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', on_pose)
    node.subscribe(Pose_V, f'/world/{args.world}/pose/info', on_pose)
    signal.signal(signal.SIGINT, lambda *_: state.__setitem__('stop', True))
    signal.signal(signal.SIGTERM, lambda *_: state.__setitem__('stop', True))
    log(f'{len(rects)} kutu engel, footprint {fp.tolist()} ({len(samples)} kenar örneği)')
    while not state['stop']:
        time.sleep(0.2)
    with lock:
        summary('SON ÖZET')
        b0, b1 = state['boxes0'] or {}, state['boxes'] or {}
        moved = [(n, math.hypot(b1[n][0] - b0[n][0], b1[n][1] - b0[n][1]))
                 for n in b0 if n in b1]
        mx = max((m for _, m in moved), default=0.0)
        log(f'Engel yer değiştirmesi: {len(moved)} kutu, maks {100 * mx:.2f} cm '
            f'({"itilen yok" if mx < 0.005 else "İTİLEN VAR"})')
        log(f'SONUÇ: {"ÇARPIŞMA YOK" if state["hit"] == 0 and mx < 0.005 else "ÇARPIŞMA VAR"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
