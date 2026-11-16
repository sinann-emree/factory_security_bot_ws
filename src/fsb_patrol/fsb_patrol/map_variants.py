"""
Genelleme kontrolü: patrol_planner'ı haritanın sentetik varyantlarında çalıştırır.

Yalnızca pgm üzerinde, Gazebo/Nav2'siz. Varyantlar: 90° ve keyfi açılarla döndürme (en yakın komşu,
dışarısı bilinmeyen), ayna, kırpma ve bunların birleşimleri.

Her varyant için kontrol: hata yok; kapsama ≥ hedef; tüm waypoint'ler traversable, maliyet ≤
max_waypoint_cost ve birbirinden ulaşılabilir (sonlu yol); koridor segmentlerinin hepsine girilir.

Kullanım:
  ros2 run fsb_patrol map_variants --map <harita.yaml> --out-dir <dizin> [--robot-params ...]
"""

import argparse
import logging
import os
import sys
import time
from typing import Callable, Dict, List, Optional

from fsb_patrol.coverage_model import GridMap, load_map, load_robot_params, save_map, UNKNOWN
from fsb_patrol.patrol_planner import plan, summarize, write_png
import numpy as np
from scipy import ndimage


def _rotate(gm: GridMap, deg: float) -> GridMap:
    g = ndimage.rotate(gm.grid, deg, reshape=True, order=0, mode='constant', cval=UNKNOWN,
                       prefilter=False)
    return GridMap(g.astype(np.int8), gm.resolution, gm.origin)


def _crop(gm: GridMap, r0: float, r1: float, c0: float, c1: float) -> GridMap:
    h, w = gm.shape
    a, b, c, d = int(r0 * h), int(r1 * h), int(c0 * w), int(c1 * w)
    x0, _ = gm.to_world(b - 1, c)
    _, y0 = gm.to_world(b - 1, c)
    origin = (float(x0) - 0.5 * gm.resolution, float(y0) - 0.5 * gm.resolution)
    return GridMap(gm.grid[a:b, c:d].copy(), gm.resolution, origin)


VARIANTS: Dict[str, Callable[[GridMap], GridMap]] = {
    'orijinal': lambda g: g,
    'dondur_90': lambda g: GridMap(np.rot90(g.grid).copy(), g.resolution, g.origin),
    'dondur_30': lambda g: _rotate(g, 30.0),
    'ayna': lambda g: GridMap(np.fliplr(g.grid).copy(), g.resolution, g.origin),
    'kirp_bati_%60': lambda g: _crop(g, 0.0, 1.0, 0.0, 0.6),
    'kirp_kd_ceyrek': lambda g: _crop(g, 0.0, 0.55, 0.45, 1.0),
    'ayna+dondur_17': lambda g: _rotate(GridMap(np.flipud(g.grid).copy(), g.resolution,
                                                g.origin), -17.0),
}


def check_variant(name: str, gm: GridMap, params_path: str,
                  png: Optional[str] = None) -> Dict[str, object]:
    p = load_robot_params(params_path)
    t0 = time.monotonic()
    res = plan(gm, p)
    s = summarize(res)
    ma = res.ma
    finite = bool(np.isfinite(res.dist).all())
    valid = all(ma.traversable[w.r, w.c] and ma.cost[w.r, w.c] <= p.max_waypoint_cost
                for w in res.waypoints)
    target = float(p.planner.get('target_coverage', 0.95))
    ok = (s['coverage_total'] >= target - 1e-9 and finite and valid
          and s['corridor_segments_entered'] == s['corridor_segments'] and s['waypoints'] >= 1)
    if png:
        write_png(res, png)
    area = float(ma.universe_free.sum() * gm.resolution ** 2)
    return {'name': name, 'ok': ok, 'sec': time.monotonic() - t0, 'area_m2': area,
            'finite': finite, 'valid': valid, **s}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--robot-params', default=None)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format='[map_variants] %(message)s')
    log = logging.getLogger('map_variants')
    log.setLevel(logging.INFO)
    params = args.robot_params
    if params is None:
        from ament_index_python.packages import get_package_share_directory
        params = os.path.join(get_package_share_directory('fsb_patrol'), 'config',
                              'robot_params.yaml')
    os.makedirs(args.out_dir, exist_ok=True)
    base = load_map(args.map)
    rows: List[Dict[str, object]] = []
    for name, fn in VARIANTS.items():
        gm = fn(base)
        ypath = os.path.join(args.out_dir, f'{name}.yaml')
        save_map(gm, ypath)              # varyant pgm/yaml olarak kaydedilir ve geri okunur
        gm = load_map(ypath)
        try:
            rows.append(check_variant(name, gm, params, os.path.join(args.out_dir, f'{name}.png')))
        except Exception as e:  # noqa: BLE001 - raporla, devam et
            rows.append({'name': name, 'ok': False, 'error': repr(e)})
    hdr = (f'{"varyant":16s} {"sonuç":6s} {"alan m²":>8s} {"wp":>4s} {"kapsama":>8s} {"dar":>7s} '
           f'{"koridor":>8s} {"tur m":>6s} {"min açık.":>9s} {"süre s":>7s}')
    log.info(hdr)
    for r in rows:
        if 'error' in r:
            log.info(f'{r["name"]:16s} HATA   {r["error"]}')
            continue
        log.info(f'{r["name"]:16s} {"OK" if r["ok"] else "KALDI":6s} {r["area_m2"]:8.1f} '
                 f'{r["waypoints"]:4d} {100 * r["coverage_total"]:7.2f}% '
                 f'{100 * r["coverage_narrow"]:6.2f}% '
                 f'{r["corridor_segments_entered"]:>3d}/{r["corridor_segments"]:<4d} '
                 f'{r["tour_length_m"]:6.1f} {r["min_clearance_m"]:9.3f} {r["sec"]:7.1f}')
    return 0 if all(r.get('ok') for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
