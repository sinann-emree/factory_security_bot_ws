"""
Dünyadan bağımsız harita + görüş modeli.

patrol_planner, coverage_monitor ve testler ortak kullanır.

Girdi yalnızca map_server formatında harita (yaml + pgm) ve robot_params.yaml'dır; kodda dünya
koordinatı yoktur. Bağımlılıklar yalnızca Ubuntu 24.04 / Jazzy apt paketleri: python3-numpy,
python3-scipy, python3-yaml.

Kavramlar
  free        : map_server eşiklerine göre serbest hücre (bilinmeyen ve dolu = engel)
  traversable : robot MERKEZİNİN durabileceği hücreler = engele uzaklık ≥ çevrel yarıçap + pay,
                en büyük bağlı bileşen (dar/geçilemeyen aralıklar böylece elenir)
  universe    : kapsanması gereken alan = traversable'a bitişik serbest bileşen; `stride`×`stride`
                hücrelik bloklar halinde (blok, içindeki herhangi bir hücre görülünce kapsanır)
  narrow      : yerel serbest genişliği < narrow_width olan hücreler (raf arası/koridor); bunlar
                yalnızca narrow_view_range içinden görülünce kapsanmış sayılır
  cost        : Nav2 InflationLayer eşdeğeri 0..254 maliyet (waypoint doğrulaması için)
"""

from dataclasses import dataclass, field
import hashlib
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage
import yaml

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
EIGHT = np.ones((3, 3), dtype=bool)


# --------------------------------------------------------------------------- harita G/Ç
@dataclass
class GridMap:
    """map_server haritası: grid[r, c] 0 serbest, 100 dolu, -1 bilinmeyen; satır 0 = üst."""

    grid: np.ndarray
    resolution: float
    origin: Tuple[float, float]
    source: str = ''
    sha256: str = ''

    @property
    def shape(self) -> Tuple[int, int]:
        return self.grid.shape

    @property
    def free(self) -> np.ndarray:
        return self.grid == FREE

    def to_world(self, r, c) -> Tuple[np.ndarray, np.ndarray]:
        h = self.grid.shape[0]
        x = self.origin[0] + (np.asarray(c) + 0.5) * self.resolution
        y = self.origin[1] + (h - 1 - np.asarray(r) + 0.5) * self.resolution
        return x, y

    def to_grid(self, x, y) -> Tuple[np.ndarray, np.ndarray]:
        h = self.grid.shape[0]
        c = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        r = (h - 1) - np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return r, c


def _read_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        data = f.read()
    tokens: List[bytes] = []
    i = 0
    while len(tokens) < 4:
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
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: yalnızca ikili PGM (P5) destekleniyor')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f'{path}: 16 bit PGM desteklenmiyor')
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w).copy()


def load_map(yaml_path: str) -> GridMap:
    """map_server yaml'ını (trinary/scale, negate, eşikler) okuyup üç değerli ızgaraya çevirir."""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    pix = _read_pgm(img_path).astype(np.float64)
    origin = [float(v) for v in meta['origin']]
    if len(origin) > 2 and abs(origin[2]) > 1e-9:
        raise ValueError('origin yaw != 0 desteklenmiyor')
    negate = int(meta.get('negate', 0))
    occ_p = pix / 255.0 if negate else (255.0 - pix) / 255.0
    grid = np.full(pix.shape, UNKNOWN, dtype=np.int8)
    grid[occ_p > float(meta['occupied_thresh'])] = OCCUPIED
    grid[occ_p < float(meta['free_thresh'])] = FREE
    with open(img_path, 'rb') as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    return GridMap(grid, float(meta['resolution']), (origin[0], origin[1]),
                   os.path.abspath(yaml_path), digest)


def save_map(gm: GridMap, yaml_path: str) -> None:
    """Trinary map_server formatında yazar (sentetik varyantlar için)."""
    base = os.path.splitext(os.path.basename(yaml_path))[0]
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), base + '.pgm')
    pix = np.full(gm.grid.shape, 205, dtype=np.uint8)
    pix[gm.grid == FREE] = 254
    pix[gm.grid == OCCUPIED] = 0
    h, w = pix.shape
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(pix.tobytes())
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'image': base + '.pgm', 'mode': 'trinary',
                        'resolution': gm.resolution,
                        'origin': [gm.origin[0], gm.origin[1], 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.196}, f)


# --------------------------------------------------------------------------- parametreler
@dataclass
class RobotParams:
    circumscribed_radius: float
    inscribed_radius: float
    safety_margin: float
    lidar_range_max: float
    camera_hfov: float
    view_range: float
    narrow_view_range: float
    narrow_width: float
    inflation_radius: float
    cost_scaling_factor: float
    max_waypoint_cost: float
    camera_frame: str = 'camera_link'
    planner: Dict[str, float] = field(default_factory=dict)

    @property
    def required_clearance(self) -> float:
        return self.circumscribed_radius + self.safety_margin

    @property
    def sensor_range(self) -> float:
        return min(self.view_range, self.lidar_range_max)


def load_robot_params(path: str) -> RobotParams:
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    r, s, c = d['robot'], d['sensors'], d['costmap']
    return RobotParams(
        circumscribed_radius=float(r['circumscribed_radius']),
        inscribed_radius=float(r['inscribed_radius']),
        safety_margin=float(r['safety_margin']),
        lidar_range_max=float(s['lidar_range_max']),
        camera_hfov=float(s['camera_hfov']),
        view_range=float(s['view_range']),
        narrow_view_range=float(s['narrow_view_range']),
        narrow_width=float(s['narrow_width']),
        inflation_radius=float(c['inflation_radius']),
        cost_scaling_factor=float(c['cost_scaling_factor']),
        max_waypoint_cost=float(c['max_waypoint_cost']),
        camera_frame=str(s.get('camera_frame', 'camera_link')),
        planner=dict(d.get('planner', {})),
    )


# --------------------------------------------------------------------------- iskelet
def _neighbors(img: np.ndarray):
    p = np.pad(img, 1)
    # Saat yönünde P2..P9 (P2 = kuzey)
    return [p[:-2, 1:-1], p[:-2, 2:], p[1:-1, 2:], p[2:, 2:],
            p[2:, 1:-1], p[2:, :-2], p[1:-1, :-2], p[:-2, :-2]]


def crossing_number(img: np.ndarray) -> np.ndarray:
    """8-komşuluk halkasındaki 0→1 geçiş sayısı (1: uç, 2: hat, ≥3: kavşak)."""
    nb = _neighbors(img.astype(np.uint8))
    a = np.zeros(img.shape, dtype=np.int16)
    for i in range(8):
        a += (nb[i] == 0) & (nb[(i + 1) % 8] == 1)
    return a


def thin(mask: np.ndarray, max_iter: int = 10000) -> np.ndarray:
    """Zhang–Suen inceltme (medial axis yaklaşıklığı), saf numpy."""
    img = mask.astype(np.uint8).copy()
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            p2, p3, p4, p5, p6, p7, p8, p9 = _neighbors(img)
            b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            seq = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            a = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8) for i in range(8))
            if step == 0:
                c = ((p2 * p4 * p6) == 0) & ((p4 * p6 * p8) == 0)
            else:
                c = ((p2 * p4 * p8) == 0) & ((p2 * p6 * p8) == 0)
            m = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & c
            if m.any():
                img[m] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


def prune(skel: np.ndarray, length_px: int) -> np.ndarray:
    """
    Morfolojik budama: uçları length_px kez sil, kalan uçlardan iskelet üzerinde geri büyüt.

    Döngüler (salonun ana halkaları) korunur; length_px'ten kısa yan dallar silinir.
    """
    x1 = skel.copy()
    for _ in range(length_px):
        ends = x1 & (crossing_number(x1) <= 1)
        if not ends.any():
            break
        x1 &= ~ends
    tips = x1 & (crossing_number(x1) <= 1)
    grow = tips.copy()
    for _ in range(length_px):
        grow = ndimage.binary_dilation(grow, structure=EIGHT) & skel
    return x1 | grow


def skeleton_nodes(skel: np.ndarray) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
    """Kavşak ve uç düğümleri; bitişik kavşak pikselleri tek düğüm (merkeze en yakın piksel)."""
    cn = crossing_number(skel) * skel
    lab, n = ndimage.label(skel & (cn >= 3), structure=EIGHT)
    junctions = []
    for k in range(1, n + 1):
        rr, cc = np.nonzero(lab == k)
        i = int(np.argmin((rr - rr.mean()) ** 2 + (cc - cc.mean()) ** 2))
        junctions.append((int(rr[i]), int(cc[i])))
    er, ec = np.nonzero(skel & (cn == 1))
    return junctions, list(zip(er.tolist(), ec.tolist()))


# --------------------------------------------------------------------------- analiz
@dataclass
class MapAnalysis:
    gm: GridMap
    params: RobotParams
    free: np.ndarray             # bool, benek temizlenmiş serbest alan
    clearance: np.ndarray        # m, en yakın serbest-olmayan hücreye uzaklık
    traversable: np.ndarray      # bool
    universe_free: np.ndarray    # bool, kapsanacak serbest hücreler
    narrow: np.ndarray           # bool
    local_width: np.ndarray      # m
    cost: np.ndarray             # 0..254
    block_id: np.ndarray         # int32, -1 = evren dışı; hücre → kapsama bloğu
    n_blocks: int
    block_narrow: np.ndarray     # bool[n_blocks], blok dar bölgede mi (çoğunluk)
    stride: int


def inflation_cost(clearance: np.ndarray, p: RobotParams) -> np.ndarray:
    """Nav2 InflationLayer: d ≤ r_iç → 253; d ≤ inflation_radius → 252·e^(−k(d−r_iç)); aksi 0."""
    cost = 252.0 * np.exp(-p.cost_scaling_factor * (clearance - p.inscribed_radius))
    cost[clearance > p.inflation_radius] = 0.0
    cost[clearance <= p.inscribed_radius] = 253.0
    cost[clearance <= 0.0] = 254.0
    return cost


def fill_unknown_specks(gm: GridMap, max_area_m2: float) -> np.ndarray:
    """
    Serbestle tamamen çevrili, YALNIZCA bilinmeyen hücrelerden oluşan küçük adacıkları serbest say.

    SLAM haritalarında ışın düşmemiş hücreler iskeleti halkalara böler ve yerel genişliği bozar.
    Dolu (occupied) benekler muhafazakâr olarak engel kalır; harita kenarına değenlere dokunulmaz.
    """
    free = gm.free.copy()
    nonfree = ~free
    lab, n = ndimage.label(nonfree, structure=EIGHT)
    if n == 0:
        return free
    idx = np.arange(1, n + 1)
    area = ndimage.sum(nonfree, lab, idx) * gm.resolution ** 2
    occ = ndimage.sum(gm.grid == OCCUPIED, lab, idx)
    border = np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))
    fill = (area < max_area_m2) & (occ == 0)
    fill[border[border > 0] - 1] = False
    free[np.isin(lab, idx[fill])] = True
    return free


def analyze(gm: GridMap, p: RobotParams, stride: Optional[int] = None) -> MapAnalysis:
    res = gm.resolution
    free = fill_unknown_specks(gm, float(p.planner.get('speckle_area', 0.05)))
    # serbest hücre → en yakın engel hücresi merkezine uzaklık
    clearance = ndimage.distance_transform_edt(free) * res
    clearance = np.where(free, clearance - 0.5 * res, 0.0)  # hücre kenarına düzeltme
    # Robot merkezi: açıklık ≥ çevrel yarıçap + pay. En büyük bağlı bileşen = devriye alanı.
    trav = free & (clearance >= p.required_clearance)
    lab, n = ndimage.label(trav, structure=EIGHT)
    if n == 0:
        raise ValueError('Haritada robotun sığabileceği serbest alan yok')
    sizes = ndimage.sum(trav, lab, index=np.arange(1, n + 1))
    traversable = lab == (int(np.argmax(sizes)) + 1)
    # Kapsanacak serbest alan: traversable ile kesişen serbest bileşen(ler)
    flab, _ = ndimage.label(free, structure=EIGHT)
    keep = np.unique(flab[traversable])
    universe_free = np.isin(flab, keep[keep > 0])
    # Yerel genişlik: serbest alan iskeletindeki en yakın noktanın açıklığının 2 katı
    skel_free = thin(universe_free)
    _, (ir, ic) = ndimage.distance_transform_edt(~skel_free, return_indices=True)
    local_width = 2.0 * clearance[ir, ic] + res
    narrow = universe_free & (local_width < p.narrow_width)
    cost = inflation_cost(clearance, p)
    # Kapsama blokları
    s = int(stride or p.planner.get('universe_stride', 2))
    h, w = free.shape
    br, bc = np.indices((h, w))
    bkey = (br // s) * ((w + s - 1) // s) + (bc // s)
    nblk_all = ((h + s - 1) // s) * ((w + s - 1) // s)
    cnt = np.bincount(bkey[universe_free], minlength=nblk_all)
    ncnt = np.bincount(bkey[narrow], minlength=nblk_all)
    valid = cnt > 0
    remap = -np.ones(nblk_all, dtype=np.int32)
    remap[valid] = np.arange(int(valid.sum()), dtype=np.int32)
    block_id = np.where(universe_free, remap[bkey], -1).astype(np.int32)
    block_narrow = (ncnt[valid] * 2 >= cnt[valid])
    return MapAnalysis(gm, p, free, clearance, traversable, universe_free, narrow, local_width,
                       cost,
                       block_id, int(valid.sum()), block_narrow, s)


# --------------------------------------------------------------------------- görüş
class Visibility:
    """
    2B ışın izleme: (r, c) konumundan sensor_range içinde görülebilen serbest hücreler.

    Işınlar serbest-olmayan (dolu/bilinmeyen) hücrede veya harita dışında durur. Dar bölge
    hücreleri yalnızca narrow_view_range içinden görülürse sayılır.
    """

    def __init__(self, ma: MapAnalysis, ray_step_deg: Optional[float] = None) -> None:
        self.ma = ma
        p = ma.params
        res = ma.gm.resolution
        step = math.radians(ray_step_deg or ma.params.planner.get('ray_step_deg', 0.5))
        self.angles = np.arange(-math.pi, math.pi, step)
        t = np.arange(1, int(math.ceil(p.sensor_range / res * 2.0)) + 1) * 0.5  # hücre birimi
        self.dist = t * res
        self.dr = -np.sin(self.angles)[:, None] * t[None, :]
        self.dc = np.cos(self.angles)[:, None] * t[None, :]
        self.free = ma.free
        self.near_ok = self.dist <= p.narrow_view_range

    def cast(self, r: float, c: float) -> Tuple[np.ndarray, np.ndarray]:
        """Her ışın için görülen blok kimlikleri. Dönüş: (ray_index[], block_id[]) (tekrarlı)."""
        h, w = self.free.shape
        rr = np.rint(r + self.dr).astype(np.int64)
        cc = np.rint(c + self.dc).astype(np.int64)
        inb = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        rr_c = np.clip(rr, 0, h - 1)
        cc_c = np.clip(cc, 0, w - 1)
        open_ = inb & self.free[rr_c, cc_c]
        vis = np.cumprod(open_, axis=1).astype(bool)
        ok = vis & (~self.ma.narrow[rr_c, cc_c] | self.near_ok[None, :])
        bid = self.ma.block_id[rr_c, cc_c]
        ok &= bid >= 0
        ray_idx = np.broadcast_to(np.arange(len(self.angles))[:, None], ok.shape)[ok]
        return ray_idx, bid[ok]

    def sector(self, ray_idx: np.ndarray, bid: np.ndarray, yaw: float) -> np.ndarray:
        """Yaw yönündeki kamera FOV'u içindeki benzersiz blok kimlikleri."""
        half = 0.5 * self.ma.params.camera_hfov
        d = np.abs((self.angles - yaw + math.pi) % (2 * math.pi) - math.pi)
        rays = d <= half
        return np.unique(bid[rays[ray_idx]])

    def seen_from_pose(self, r: float, c: float, yaw: float) -> np.ndarray:
        ray_idx, bid = self.cast(r, c)
        return self.sector(ray_idx, bid, yaw)
