#!/usr/bin/env python3
"""
Kaydedilmiş occupancy haritasının (map_saver .yaml/.pgm) sayısal kalite kontrolü.

Referans: dünya SDF'indeki kutu engeller.

Varsayım: harita çerçevesi = dünya çerçevesi (robot (0,0,0)'da spawn, SLAM oradan başlar).
Ölçütler:
  * Doğruluk: her dolu hücrenin en yakın gerçek engel yüzeyine uzaklığı (medyan, p95, maks.);
    > 10 cm uzak dolu hücreler = çift duvar / hayalet engel göstergesi.
  * Global hizalama: haritayı dünyaya en iyi oturtan (dθ, dx, dy) ızgara araması; 0'a yakın olmalı
    (harita dönmemiş / kaymamış).
  * Duvar kalınlığı: 4 dış duvarın iç yüzünde sütun başına dolu hücre sayısı (çift duvar → artar).
  * Kapsama: salon içinden görülebilen engel yüzeylerinin (5 cm aralıklı örnekler) haritada
    ≤ 10 cm içinde dolu hücreyle karşılanma oranı.
Kullanım:
  ros2 run fsb_gazebo map_quality_check.py --map <yol>/factory.yaml
"""

import argparse
import math
import os
import sys
from typing import List, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
import rclpy.logging
import rclpy.utilities
import yaml

Rect = Tuple[str, float, float, float, float]


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


def read_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:  # P5 width height maxval (yorumları atla)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    assert tokens[0] == b'P5', 'yalnızca ikili PGM (P5)'
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def boundary_distance(px: np.ndarray, py: np.ndarray, rects: List[Rect]) -> np.ndarray:
    best = np.full(px.shape, np.inf)
    for _, cx, cy, hx, hy in rects:
        dx = np.abs(px - cx) - hx
        dy = np.abs(py - cy) - hy
        sd = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0)
        best = np.minimum(best, np.abs(sd))
    return best


def main(argv=None) -> int:
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map_saver .yaml')
    ap.add_argument('--sdf', default=os.path.join(
        get_package_share_directory('fsb_gazebo'), 'worlds', 'factory.sdf'))
    ap.add_argument('--hall-x', type=float, default=10.0, help='salon iç yarı uzunluğu x')
    ap.add_argument('--hall-y', type=float, default=7.0, help='salon iç yarı uzunluğu y')
    args = ap.parse_args(argv)

    rclpy.init()
    log = rclpy.logging.get_logger('fsb_map_quality')
    try:
        with open(args.map, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        img = read_pgm(os.path.join(os.path.dirname(args.map), meta['image']))
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        assert abs(float(meta['origin'][2])) < 1e-9, 'origin yaw != 0 desteklenmiyor'
        neg = int(meta.get('negate', 0))
        occ_t = float(meta['occupied_thresh'])
        free_t = float(meta['free_thresh'])
        p = img.astype(float) / 255.0
        occ_prob = p if neg else 1.0 - p
        occ = occ_prob > occ_t
        free = occ_prob < free_t
        h, w = img.shape
        rows, cols = np.nonzero(occ)
        px = ox + (cols + 0.5) * res
        py = oy + (h - 1 - rows + 0.5) * res
        rects = load_boxes(args.sdf)

        log.info(f'Harita: {w}x{h} hücre, {res} m/hücre, origin=({ox:.2f}, {oy:.2f})  '
                 f'dolu={occ.sum()}  boş={free.sum()}  bilinmeyen={(~occ & ~free).sum()}')

        # --- Doğruluk ---
        d = boundary_distance(px, py, rects)
        log.info('DOĞRULUK (dolu hücre → en yakın gerçek yüzey):')
        log.info(f'  medyan={np.median(d) * 100:.2f} cm  p95={np.percentile(d, 95) * 100:.2f} cm  '
                 f'maks={d.max() * 100:.2f} cm')
        for lim in (0.05, 0.10):
            log.info(f'  ≤ {lim * 100:.0f} cm: %{100.0 * (d <= lim).mean():.2f}')
        far = d > 0.10
        log.info(f'  > 10 cm (çift duvar / hayalet göstergesi): {far.sum()} hücre '
                 f'(%{100.0 * far.mean():.2f})')

        # --- Global hizalama ---
        best = (np.inf, 0.0, 0.0, 0.0)
        for dth in np.radians(np.arange(-1.0, 1.0001, 0.2)):
            c, s = math.cos(dth), math.sin(dth)
            rx, ry = c * px - s * py, s * px + c * py
            for dx in np.arange(-0.10, 0.1001, 0.02):
                for dy in np.arange(-0.10, 0.1001, 0.02):
                    m = float(np.median(boundary_distance(rx + dx, ry + dy, rects)))
                    if m < best[0]:
                        best = (m, math.degrees(dth), dx, dy)
        log.info(f'GLOBAL HİZALAMA: en iyi dθ={best[1]:+.1f}°  dx={best[2] * 100:+.0f} cm  '
                 f'dy={best[3] * 100:+.0f} cm  (medyan {best[0] * 100:.2f} cm; '
                 f'ızgara ±1°/0.2°, ±10 cm/2 cm)')

        # --- Duvar kalınlığı (iç yüzlerde) ---
        log.info('DUVAR KALINLIĞI (iç yüz ±0.5 m bandında, sütun başına dolu hücre):')
        hx, hy = args.hall_x, args.hall_y
        walls = {
            'kuzey': (np.abs(px) < hx - 1.0) & (np.abs(py - hy) < 0.5),
            'güney': (np.abs(px) < hx - 1.0) & (np.abs(py + hy) < 0.5),
            'doğu': (np.abs(py) < hy - 1.0) & (np.abs(px - hx) < 0.5),
            'batı': (np.abs(py) < hy - 1.0) & (np.abs(px + hx) < 0.5),
        }
        for name, sel in walls.items():
            along = px[sel] if name in ('kuzey', 'güney') else py[sel]
            if along.size == 0:
                log.info(f'  {name:6s}: dolu hücre yok')
                continue
            _, counts = np.unique(np.round(along / res).astype(int), return_counts=True)
            log.info(f'  {name:6s}: sütun={counts.size}  kalınlık medyan={np.median(counts):.0f}  '
                     f'maks={counts.max()} hücre  '
                     f'(≥3 hücre olan sütun: %{100.0 * (counts >= 3).mean():.1f})')

        # --- Kapsama ---
        samples = []
        for _, cx, cy, ex, ey in rects:
            for t in np.arange(-ex, ex + 1e-9, 0.05):
                samples += [(cx + t, cy - ey), (cx + t, cy + ey)]
            for t in np.arange(-ey, ey + 1e-9, 0.05):
                samples += [(cx - ex, cy + t), (cx + ex, cy + t)]
        s = np.array(samples)
        inside = (np.abs(s[:, 0]) <= hx + 1e-6) & (np.abs(s[:, 1]) <= hy + 1e-6)
        s = s[inside]
        # Bir yüzey örneği başka bir kutunun içinde kalıyorsa (bitişik yüzey) görülemez → çıkar
        hidden = np.zeros(len(s), dtype=bool)
        for _, cx, cy, ex, ey in rects:
            hidden |= (np.abs(s[:, 0] - cx) < ex - 1e-6) & (np.abs(s[:, 1] - cy) < ey - 1e-6)
        s = s[~hidden]
        c = np.floor((s[:, 0] - ox) / res).astype(int)
        r = h - 1 - np.floor((s[:, 1] - oy) / res).astype(int)
        k = 2  # ±2 hücre = 10 cm
        hit = np.zeros(len(s), dtype=bool)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                rr, cc = r + dr, c + dc
                ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
                hit[ok] |= occ[rr[ok], cc[ok]]
        log.info(f'KAPSAMA: {hit.sum()}/{len(s)} yüzey örneği haritada (≤10 cm) '
                 f'→ %{100.0 * hit.mean():.1f}')
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
