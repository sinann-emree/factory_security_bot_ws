r"""
patrol_planner: harita (yaml/pgm) + robot_params.yaml → kapsamlı devriye waypoint yaml'ı.

Dünyadan bağımsızdır; kodda dünya koordinatı yoktur. Yeni harita geldiğinde yalnızca bu
çalıştırılır.

Adımlar
  1. Serbest alan: serbestle çevrili yalnız-bilinmeyen adacıklar temizlenir; robot merkezi için
     açıklık ≥ çevrel yarıçap + güvenlik payı olan hücreler (traversable) — daha dar aralıklar
     elenir, en büyük bileşen alınır.
  2. Medial axis (Zhang–Suen) iskeleti, kısa dallar budanır; kavşak ve çıkmaz uç düğümleri +
     iskelet boyunca sample_step aralıklı ek noktalar = aday konumlar.
  3. Koridor segmentleri (dar bölgedeki, kavşak dairelerinin dışında kalan ≥ corridor_min_length
     iskelet parçaları: raf araları, duvar şeritleri) — her birinin İÇİNE bir waypoint zorunlu;
     böylece devriye aralara fiilen girer (ağızdan bakmak yetmez).
  4. Her aday × yaw_sectors yönelim için kamera FOV'u ve menzil içinde ışın izlemeyle görülen alan;
     koridor tohumlarından sonra greedy (tembel) set cover ile serbest alanın ≥ target_coverage'ı
     kapsanana dek seç, sonra gereksizleri at. Yaw = seçilen sektör, yani o noktadan henüz
     kapsanmamış alana bakan yön.
  5. Waypoint doğrulaması: Nav2 InflationLayer eşdeğeri maliyet ≤ max_waypoint_cost ve açıklık;
     --cost nav2 iken ayrıca canlı /global_costmap/costmap'te kontrol.
  6. Maliyet matrisi: --cost nav2 → Nav2 planner_server'ın ComputePathToPose yol uzunlukları
     (çalışan Nav2 ya da `ros2 launch fsb_patrol planner_backend.launch.py`); --cost grid → aynı
     traversable ızgarada Dijkstra (Gazebo/Nav2'siz, sentetik testler için).
  7. TSP (Held–Karp / 2-opt + Or-opt) ile kapalı tur sırası.

Kullanım:
  ros2 run fsb_patrol patrol_planner --map <harita.yaml> --out <waypoints.yaml> \\
      [--robot-params <robot_params.yaml>] [--cost nav2|grid] [--png <görsel.png>]
"""

import argparse
from dataclasses import dataclass
import datetime
import heapq
import logging
import math
import os
import sys
from typing import Callable, Dict, List, Optional, Tuple

from fsb_patrol import tsp
from fsb_patrol.coverage_model import (analyze, EIGHT, GridMap, load_map, load_robot_params,
                                       MapAnalysis, prune, RobotParams, skeleton_nodes, thin,
                                       Visibility)
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
import yaml

LOG = logging.getLogger('patrol_planner')


@dataclass
class Candidate:
    r: int
    c: int
    kind: str  # junction | endpoint | sample | corridor
    segment: int = -1  # içinde bulunduğu koridor segmenti (-1: yok)


@dataclass
class Waypoint:
    r: int
    c: int
    yaw: float
    kind: str
    blocks: np.ndarray
    new_blocks: int = 0
    segment: int = -1


@dataclass
class PlanResult:
    ma: MapAnalysis
    skeleton: np.ndarray
    candidates: List[Candidate]
    waypoints: List[Waypoint]          # tur sırasında
    dist: np.ndarray                   # tur sırasında yol uzunlukları (m)
    covered: np.ndarray                # bool[n_blocks]
    reachable_max: float               # tüm adaylarla erişilebilir en yüksek kapsama
    cost_source: str
    dropped: List[str]
    segments: List[np.ndarray]         # koridor segmentleri (piksel indeksleri, (k, 2))


# --------------------------------------------------------------------------- adaylar
def corridor_segments(ma: MapAnalysis, skel: np.ndarray,
                      junctions: List[Tuple[int, int]]) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    Dar bölgedeki iskelet parçaları (kavşak çevreleri çıkarılmış), ≥ corridor_min_length.

    Raf araları, duvar boyu şeritler ve ara koridorlar bu tanıma girer; her biri devriyenin
    fiilen içine girmesi gereken bir bölgedir. Dönüş: (etiket ızgarası, segment piksel listeleri).
    """
    p, res = ma.params, ma.gm.resolution
    # Her kavşağın etki alanı = o noktadaki iç teğet daire (yarıçap = açıklık)
    jmask = np.zeros(skel.shape, dtype=bool)
    h, w = skel.shape
    for r, c in junctions:
        rad = max(1.0, ma.clearance[r, c] / res)
        k = int(math.ceil(rad))
        r0, r1, c0, c1 = max(0, r - k), min(h, r + k + 1), max(0, c - k), min(w, c + k + 1)
        yy, xx = np.ogrid[r0:r1, c0:c1]
        jmask[r0:r1, c0:c1] |= (yy - r) ** 2 + (xx - c) ** 2 <= rad * rad
    seg_px = skel & ma.narrow & ~jmask
    lab, n = ndimage.label(seg_px, structure=EIGHT)
    min_px = p.planner.get('corridor_min_length', 2.0) / res
    out = np.zeros(skel.shape, dtype=np.int32) - 1
    segs: List[np.ndarray] = []
    for k in range(1, n + 1):
        px = np.argwhere(lab == k)
        if len(px) >= min_px:
            out[lab == k] = len(segs)
            segs.append(px)
    return out, segs


def make_candidates(ma: MapAnalysis) -> Tuple[np.ndarray, List[Candidate], List[np.ndarray]]:
    p, res = ma.params, ma.gm.resolution
    # İskelet topolojisi için küçük delikleri doldur: tek hücrelik dolu benekler iskelette sahte
    # halka/kavşak üretir ve koridorları böler. Gerçek engeller (kolon vb.) eşikten büyüktür.
    # Adaylar yine gerçek traversable + maliyet kontrolünden geçer.
    trav = ma.traversable
    holes = ndimage.binary_fill_holes(trav) & ~trav
    hlab, nh = ndimage.label(holes, structure=EIGHT)
    if nh:
        area = ndimage.sum(holes, hlab, np.arange(1, nh + 1)) * res * res
        small = np.arange(1, nh + 1)[area < p.planner.get('hole_fill_area', 0.5)]
        trav = trav | np.isin(hlab, small)
    skel = prune(thin(trav), max(1, int(round(p.planner.get('prune_length', 0.6) / res))))
    junctions, ends = skeleton_nodes(skel)
    seg_lab, segs = corridor_segments(ma, skel, junctions)
    cands: List[Candidate] = [Candidate(r, c, 'junction') for r, c in junctions]
    cands += [Candidate(r, c, 'endpoint') for r, c in ends]
    # İskelet boyunca ek noktalar: sample_step × sample_step bloklarda en geniş açıklıklı piksel
    step = max(1, int(round(p.planner.get('sample_step', 1.0) / res)))
    rr, cc = np.nonzero(skel)
    key = (rr // step) * 100000 + (cc // step)
    order = np.lexsort((-ma.clearance[rr, cc], key))
    first = np.ones(len(order), dtype=bool)
    first[1:] = key[order][1:] != key[order][:-1]
    for i in order[first]:
        cands.append(Candidate(int(rr[i]), int(cc[i]), 'sample'))
    # Düğümlere 0.3 m'den yakın örnekleri at (düğüm önceliği), maliyet/açıklık filtresi
    min_sep = 0.3 / res
    kept: List[Candidate] = []
    for cd in cands:
        if ma.cost[cd.r, cd.c] > p.max_waypoint_cost or not ma.traversable[cd.r, cd.c]:
            continue
        if cd.kind == 'sample' and any(k.kind != 'sample' and
                                       (k.r - cd.r) ** 2 + (k.c - cd.c) ** 2 < min_sep ** 2
                                       for k in kept):
            continue
        kept.append(cd)
    # Koridor segmentleri: içindeki adayları etiketle; adayı olmayan segmente orta nokta ekle
    seg_near = ndimage.grey_dilation(seg_lab + 1, footprint=EIGHT) - 1
    for cd in kept:
        cd.segment = int(seg_near[cd.r, cd.c])
    have = {cd.segment for cd in kept}
    for k, px in enumerate(segs):
        if k in have:
            continue
        ok = [q for q in px if ma.cost[q[0], q[1]] <= p.max_waypoint_cost]
        if ok:
            r, c = ok[len(ok) // 2]
            kept.append(Candidate(int(r), int(c), 'corridor', k))
    return skel, kept, segs


# --------------------------------------------------------------------------- set cover
def greedy_cover(ma: MapAnalysis, cands: List[Candidate], target: float, n_segments: int = 0
                 ) -> Tuple[List[Waypoint], np.ndarray, float]:
    p = ma.params
    vis = Visibility(ma)
    nsec = int(p.planner.get('yaw_sectors', 12))
    yaws = [-math.pi + (k + 0.5) * 2 * math.pi / nsec for k in range(nsec)]
    sets: List[Tuple[int, float, np.ndarray]] = []
    union = np.zeros(ma.n_blocks, dtype=bool)
    for i, cd in enumerate(cands):
        ray_idx, bid = vis.cast(cd.r, cd.c)
        for y in yaws:
            s = vis.sector(ray_idx, bid, y)
            if len(s):
                sets.append((i, y, s))
                union[s] = True
    reachable_max = float(union.mean())
    LOG.info(f'{len(cands)} aday × {nsec} yön = {len(sets)} görüş kümesi; '
             f'tüm adaylarla erişilebilir kapsama %{100 * reachable_max:.2f}')
    need = int(math.ceil(target * ma.n_blocks))
    if union.sum() < need:
        LOG.warning(f'Hedef %{100 * target:.0f} erişilemez; en fazla %{100 * reachable_max:.2f}')
        need = int(union.sum())
    covered = np.zeros(ma.n_blocks, dtype=bool)
    chosen: List[int] = []
    # Tohum: her koridor segmentinin içine bir waypoint (segmentte en çok yeni alan gören aday+yön)
    seeds = set()
    for seg in range(n_segments):
        opts = [k for k, (i, _, _s) in enumerate(sets) if cands[i].segment == seg]
        if not opts:
            LOG.warning(f'Koridor segmenti {seg}: geçerli aday yok (maliyet/açıklık)')
            continue
        k = max(opts, key=lambda k: int((~covered[sets[k][2]]).sum()))
        chosen.append(k)
        seeds.add(k)
        covered[sets[k][2]] = True
    LOG.info(f'Koridor tohumları: {len(seeds)}/{n_segments} segment, '
             f'kapsama %{100 * covered.mean():.2f}')
    bonus = {'junction': 1.03, 'endpoint': 1.03, 'sample': 1.0, 'corridor': 1.0}  # eşitlikte düğüm
    heap = [(-len(s) * bonus[cands[i].kind], k) for k, (i, _, s) in enumerate(sets)]
    heapq.heapify(heap)
    while covered.sum() < need and heap:
        _, k = heapq.heappop(heap)
        i, _, s = sets[k]
        gain = int((~covered[s]).sum()) * bonus[cands[i].kind]
        if gain <= 0:
            continue
        if heap and gain < -heap[0][0]:
            heapq.heappush(heap, (-gain, k))  # tembel güncelleme
            continue
        chosen.append(k)
        covered[s] = True
    # Gereksizleri ayıkla: en az katkılıdan başlayarak, çıkarınca hedef korunuyorsa çıkar
    count = np.zeros(ma.n_blocks, dtype=np.int32)
    for k in chosen:
        count[sets[k][2]] += 1
    removed = True
    while removed:
        removed = False
        uniq = sorted((k for k in chosen if k not in seeds),
                      key=lambda k: int((count[sets[k][2]] == 1).sum()))
        for k in uniq:
            lost = int((count[sets[k][2]] == 1).sum())
            if int((count > 0).sum()) - lost >= need:
                count[sets[k][2]] -= 1
                chosen.remove(k)
                removed = True
                break
    covered = count > 0
    wps = []
    for k in chosen:
        i, y, s = sets[k]
        wps.append(Waypoint(cands[i].r, cands[i].c, y, cands[i].kind, s,
                            int((count[s] == 1).sum()), cands[i].segment if k in seeds else -1))
    LOG.info(f'Set cover: {len(wps)} waypoint, kapsama %{100 * covered.mean():.2f}')
    return wps, covered, reachable_max


# --------------------------------------------------------------------------- maliyetler
def grid_path_lengths(ma: MapAnalysis, pts: List[Tuple[int, int]]) -> np.ndarray:
    """Traversable ızgarada 8-komşulu Dijkstra yol uzunlukları (m)."""
    trav = ma.traversable
    h, w = trav.shape
    idx = -np.ones((h, w), dtype=np.int64)
    idx[trav] = np.arange(int(trav.sum()))
    rows, cols, wts = [], [], []
    res = ma.gm.resolution
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):  # (r, c) ↔ (r + dr, c + dc)
        a = idx[0:h - dr, max(0, -dc):w - max(0, dc)]
        b = idx[dr:h, max(0, dc):w - max(0, -dc)]
        m = (a >= 0) & (b >= 0)
        rows.append(a[m])
        cols.append(b[m])
        wts.append(np.full(int(m.sum()), res * math.hypot(dr, dc)))
    n = int(trav.sum())
    g = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(n, n)).tocsr()
    src = [int(idx[r, c]) for r, c in pts]
    d = dijkstra(g, directed=False, indices=src)
    return d[:, src]


CostFn = Callable[[List[Tuple[float, float, float]]], np.ndarray]


# --------------------------------------------------------------------------- plan
def plan(gm: GridMap, p: RobotParams, cost_fn: Optional[CostFn] = None,
         target: Optional[float] = None,
         costmap_check: Optional[Callable[[float, float], Optional[int]]] = None) -> PlanResult:
    target = float(target if target is not None else p.planner.get('target_coverage', 0.95))
    ma = analyze(gm, p)
    skel, cands, segs = make_candidates(ma)
    nj = sum(c.kind == 'junction' for c in cands)
    ne = sum(c.kind == 'endpoint' for c in cands)
    LOG.info(f'İskelet {int(skel.sum())} px; aday: {nj} kavşak, {ne} uç, {len(cands) - nj - ne} '
             f'örnek/koridor; {len(segs)} koridor segmenti')
    wps, covered, reach = greedy_cover(ma, cands, target, len(segs))
    dropped: List[str] = []
    # Canlı costmap doğrulaması (nav2 modu)
    if costmap_check is not None:
        ok = []
        for w in wps:
            x, y = gm.to_world(w.r, w.c)
            v = costmap_check(float(x), float(y))
            if v is not None and v >= 0 and v > _occ_from_cost(p.max_waypoint_cost):
                dropped.append(f'({float(x):.2f},{float(y):.2f}) costmap={v}')
                LOG.warning(f'Waypoint ({float(x):.2f},{float(y):.2f}) canlı costmap değeri '
                            f'{v} → atıldı')
                continue
            ok.append(w)
        LOG.info(f'Canlı Nav2 global costmap kontrolü: {len(ok)}/{len(wps)} waypoint geçti '
                 f'(eşik OccupancyGrid {_occ_from_cost(p.max_waypoint_cost)})')
        wps = ok
    poses = []
    for w in wps:
        x, y = gm.to_world(w.r, w.c)
        poses.append((float(x), float(y), w.yaw))
    if cost_fn is None:
        d = grid_path_lengths(ma, [(w.r, w.c) for w in wps])
        source = 'grid'
    else:
        d = cost_fn(poses)
        source = 'nav2'
    # Ulaşılamayanları çıkar (en çok sonsuz bağlantısı olandan başlayarak)
    keep = list(range(len(wps)))
    while keep:
        sub = d[np.ix_(keep, keep)]
        bad = (~np.isfinite(sub)).sum(axis=1)
        if bad.max() == 0:
            break
        k = keep[int(np.argmax(bad))]
        x, y, _ = poses[k]
        dropped.append(f'({x:.2f},{y:.2f}) ulaşılamaz')
        LOG.warning(f'Waypoint ({x:.2f},{y:.2f}) ulaşılamaz → atıldı')
        keep.remove(k)
    wps = [wps[k] for k in keep]
    d = d[np.ix_(keep, keep)]
    covered = np.zeros(ma.n_blocks, dtype=bool)
    for w in wps:
        covered[w.blocks] = True
    order = tsp.solve(0.5 * (d + d.T)) if len(wps) > 1 else list(range(len(wps)))
    wps = [wps[k] for k in order]
    d = d[np.ix_(order, order)]
    return PlanResult(ma, skel, cands, wps, d, covered, reach, source, dropped, segs)


def _occ_from_cost(cost: float) -> int:
    """Nav2 costmap 0..252 → OccupancyGrid 0..100 (nav2_costmap_2d cost_translation_table)."""
    if cost <= 0:
        return 0
    return int(1 + (97 * (cost - 1)) / 251)


# --------------------------------------------------------------------------- rapor / çıktı
def summarize(res: PlanResult) -> Dict[str, float]:
    ma = res.ma
    nb = ma.block_narrow
    out = {
        'waypoints': len(res.waypoints),
        'coverage_total': float(res.covered.mean()),
        'coverage_narrow': float(res.covered[nb].mean()) if nb.any() else 1.0,
        'coverage_wide': float(res.covered[~nb].mean()) if (~nb).any() else 1.0,
        'reachable_max': res.reachable_max,
        'tour_length_m': float(sum(res.dist[i, (i + 1) % len(res.waypoints)]
                                   for i in range(len(res.waypoints)))) if res.waypoints else 0.0,
        'waypoints_in_narrow': int(sum(bool(ma.narrow[w.r, w.c]) for w in res.waypoints)),
        'min_clearance_m': float(min(ma.clearance[w.r, w.c] for w in res.waypoints)),
        'max_cost': float(max(ma.cost[w.r, w.c] for w in res.waypoints)),
        'narrow_area_ratio': float(ma.block_narrow.mean()),
        'corridor_segments': len(res.segments),
        'corridor_segments_entered': len({w.segment for w in res.waypoints if w.segment >= 0}),
    }
    return out


def _rel_to(target: str, out_path: str) -> str:
    """
    Return ``target`` relative to the directory of ``out_path``.

    Kaynak yolları yalnızca bilgi amaçlıdır (haritanın kimliği sha256 ile doğrulanır);
    göreli yazılınca dosya makineye/kullanıcıya özel mutlak yol içermez.
    """
    return os.path.relpath(os.path.abspath(target),
                           os.path.dirname(os.path.abspath(out_path)))


def write_yaml(res: PlanResult, path: str, params_path: str) -> None:
    gm = res.ma.gm
    s = summarize(res)
    wps = []
    for i, w in enumerate(res.waypoints):
        x, y = gm.to_world(w.r, w.c)
        wps.append({'name': f'W{i + 1:02d}', 'x': round(float(x), 3), 'y': round(float(y), 3),
                    'yaw': round(float(w.yaw), 4), 'kind': w.kind,
                    'narrow': bool(res.ma.narrow[w.r, w.c]),
                    'corridor_segment': int(w.segment),
                    'clearance_m': round(float(res.ma.clearance[w.r, w.c]), 3),
                    'cost': int(res.ma.cost[w.r, w.c]),
                    'unique_blocks': int(w.new_blocks)})
    doc = {
        'generated_by': 'fsb_patrol/patrol_planner',
        'generated_at': datetime.datetime.now().isoformat(timespec='seconds'),
        'frame_id': 'map',
        'map': {'yaml': _rel_to(gm.source, path), 'sha256': gm.sha256},
        'robot_params': _rel_to(params_path, path),
        'cost_source': res.cost_source,
        'summary': {k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()},
        'dropped': res.dropped,
        'waypoints': wps,
        'path_lengths_m': [[round(float(v), 3) for v in row] for row in res.dist],
    }
    with open(path, 'w', encoding='utf-8') as f:
        f.write("# Otomatik üretildi — elle düzenlemeyin; harita değişince patrol_planner'ı "
                'yeniden çalıştırın.\n')
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, width=200)


def write_png(res: PlanResult, path: str, title: str = '') -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Wedge

    ma, gm = res.ma, res.ma.gm
    h, w = gm.shape
    ext = [gm.origin[0], gm.origin[0] + w * gm.resolution,
           gm.origin[1], gm.origin[1] + h * gm.resolution]
    img = np.full((h, w, 3), 0.55)
    img[ma.free] = (1.0, 1.0, 1.0)
    img[ma.narrow] = (0.93, 0.95, 1.0)
    cov_cells = np.zeros((h, w), dtype=bool)
    cov_cells[ma.block_id >= 0] = res.covered[ma.block_id[ma.block_id >= 0]]
    img[ma.universe_free & ~cov_cells] = (1.0, 0.75, 0.75)
    img[gm.grid == 100] = (0.1, 0.1, 0.1)
    img[res.skeleton] = (0.6, 0.75, 0.6)
    for px in res.segments:  # koridor segmentleri (içine girilmesi zorunlu)
        img[px[:, 0], px[:, 1]] = (0.55, 0.2, 0.75)
    fig, ax = plt.subplots(figsize=(13, 9), dpi=120)
    ax.imshow(img, extent=ext, interpolation='nearest')
    rng = ma.params.sensor_range
    hf = math.degrees(ma.params.camera_hfov) / 2
    xs, ys = [], []
    for i, wp in enumerate(res.waypoints):
        x, y = gm.to_world(wp.r, wp.c)
        xs.append(float(x))
        ys.append(float(y))
        yd = math.degrees(wp.yaw)
        ax.add_patch(Wedge((x, y), min(rng, 1.2), yd - hf, yd + hf, alpha=0.25,
                           color='#e6a100', lw=0))
        ax.annotate(f'{i + 1}', (x, y), fontsize=7, ha='center', va='center',
                    color='white', weight='bold',
                    bbox={'boxstyle': 'circle,pad=0.2', 'lw': 0,
                          'fc': '#7b2cbf' if wp.segment >= 0 else
                          ('#c0392b' if wp.kind in ('junction', 'endpoint') else '#2563eb')})
    if xs:
        ax.plot(xs + xs[:1], ys + ys[:1], '-', color='#2563eb', lw=0.8, alpha=0.6)
    s = summarize(res)
    ax.set_title(title or (
        f'{s["waypoints"]} waypoint · kapsama %{100 * s["coverage_total"]:.1f} '
        f'(dar %{100 * s["coverage_narrow"]:.1f}) · tur {s["tour_length_m"]:.0f} m '
        f'({res.cost_source}) · koridor '
        f'{s["corridor_segments_entered"]}/{s["corridor_segments"]}\n'
        'mor: koridor segmenti/tohum · kırmızı: kavşak/uç · mavi: örnek · pembe: kapsanmayan'))
    free_rows, free_cols = np.nonzero(ma.universe_free)
    fx, fy = gm.to_world(free_rows, free_cols)
    ax.set_xlim(float(fx.min()) - 0.5, float(fx.max()) + 0.5)
    ax.set_ylim(float(fy.min()) - 0.5, float(fy.max()) + 0.5)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# --------------------------------------------------------------------------- Nav2 arka ucu
def _nav2_backend(log_ros):
    """(cost_fn, costmap_check, shutdown) — ROS yalnızca --cost nav2 iken içe aktarılır."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=sys.argv)
    node = Node('fsb_patrol_planner')
    client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    grid_holder: Dict[str, OccupancyGrid] = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                             lambda m: grid_holder.__setitem__('g', m), qos)
    if not client.wait_for_server(timeout_sec=30.0):
        raise RuntimeError('compute_path_to_pose action sunucusu yok '
                           '(Nav2 / planner_backend çalışıyor mu?)')
    t_end = node.get_clock().now().nanoseconds + 15e9
    while 'g' not in grid_holder and node.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'g' not in grid_holder:
        node.get_logger().warning(
            '/global_costmap/costmap alınamadı; canlı costmap kontrolü atlanıyor')

    def pose(x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def cost_fn(poses: List[Tuple[float, float, float]]) -> np.ndarray:
        n = len(poses)
        d = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i == j or math.hypot(poses[i][0] - poses[j][0],
                                        poses[i][1] - poses[j][1]) < 1e-3:
                    continue  # aynı konum (farklı yaw'lı iki görüş): yol uzunluğu 0
                g = ComputePathToPose.Goal()
                g.start, g.goal, g.use_start = pose(*poses[i]), pose(*poses[j]), True
                fut = client.send_goal_async(g)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                gh = fut.result()
                if gh is None or not gh.accepted:
                    d[i, j] = np.inf
                    continue
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(node, rf, timeout_sec=10.0)
                r = rf.result()
                pts = r.result.path.poses if r is not None else []
                if r is None or r.status != 4 or len(pts) < 2:  # 4 = SUCCEEDED
                    d[i, j] = np.inf
                    continue
                xy = np.array([[q.pose.position.x, q.pose.position.y] for q in pts])
                d[i, j] = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
            node.get_logger().info(f'Nav2 yol uzunlukları: {i + 1}/{n}')
        return d

    def costmap_check(x: float, y: float) -> Optional[int]:
        g = grid_holder.get('g')
        if g is None:
            return None
        c = int((x - g.info.origin.position.x) / g.info.resolution)
        r = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= c < g.info.width and 0 <= r < g.info.height):
            return 100
        return int(g.data[r * g.info.width + c])

    def shutdown() -> None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return cost_fn, costmap_check, shutdown


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    try:
        from rclpy.utilities import remove_ros_args
        args_in = remove_ros_args(argv)[1:]
    except ImportError:
        args_in = argv[1:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map_server yaml')
    ap.add_argument('--out', required=True, help='çıktı waypoint yaml')
    ap.add_argument('--robot-params', default=None,
                    help='robot_params.yaml (varsayılan: fsb_patrol/config/robot_params.yaml)')
    ap.add_argument('--cost', choices=('nav2', 'grid'), default='grid')
    ap.add_argument('--target', type=float, default=None, help='kapsama hedefi (0..1)')
    ap.add_argument('--png', default=None, help='görsel çıktı (isteğe bağlı)')
    args = ap.parse_args(args_in)
    logging.basicConfig(level=logging.INFO, format='[patrol_planner] %(message)s')
    params_path = args.robot_params
    if params_path is None:
        from ament_index_python.packages import get_package_share_directory
        params_path = os.path.join(get_package_share_directory('fsb_patrol'),
                                   'config', 'robot_params.yaml')
    gm = load_map(args.map)
    p = load_robot_params(params_path)
    shutdown = None
    cost_fn = check = None
    if args.cost == 'nav2':
        cost_fn, check, shutdown = _nav2_backend(LOG)
    try:
        res = plan(gm, p, cost_fn, args.target, check)
    finally:
        if shutdown:
            shutdown()
    write_yaml(res, args.out, params_path)
    if args.png:
        write_png(res, args.png)
    s = summarize(res)
    LOG.info('ÖZET: ' + '  '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}'
                                  for k, v in s.items()))
    LOG.info(f'yazıldı: {args.out}')
    target = float(args.target if args.target is not None
                   else p.planner.get('target_coverage', 0.95))
    return 0 if s['coverage_total'] >= target - 1e-9 else 2


if __name__ == '__main__':
    raise SystemExit(main())
