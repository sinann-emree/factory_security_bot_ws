"""
Olay geometrisi (ROS'tan bağımsız, saf fonksiyonlar).

  locate_from_scan  tespit kutusunun yatay açı aralığındaki LiDAR noktalarından kişinin konumu
  ground_ray        yedek: kutunun alt-orta pikselinden zemin düzlemine ışın
  plan_viewpoints   kişinin çevresinde, costmap'te boş ve görüş hattı açık bakış noktaları
  check_view        "kişi artık yok" kararı için: konum kamera görüş alanında mı, görüş hattı
                    açık mı, konumda hâlâ bir cisim var mı (LiDAR)

Çerçeveler: "base" = base_footprint (x ileri, y sol). Kamera optik çerçevesi: z ileri, x sağ,
y aşağı. 2B dönüşümler (x, y, yaw) üçlüsüdür.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

Pose2D = Tuple[float, float, float]


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_frame(p: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """Çocuk çerçevedeki (x, y) noktasını üst çerçeveye taşır; p = çocuğun üst çerçevedeki pozu."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return float(p[0] + c * x - s * y), float(p[1] + s * x + c * y)


def scan_points(ranges: Sequence[float], angle_min: float, angle_inc: float,
                lidar_in_base: Pose2D, r_min: float, r_max: float,
                keep_far: bool = False) -> np.ndarray:
    """
    Geçerli tarama noktaları, base çerçevesinde (N, 2).

    keep_far: menzil dışı / dönüşsüz ışınlar atılmaz, r_max uzaklığında nokta sayılır
    ("o yönde r_max'a kadar engel yok" bilgisi görüş doğrulamasında gereklidir).
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(r.size)
    if keep_far:
        r = np.where(np.isnan(r), -1.0, np.minimum(r, r_max))
    ok = np.isfinite(r) & (r >= r_min) & (r <= r_max)
    r, a = r[ok], a[ok]
    c, s = math.cos(lidar_in_base[2]), math.sin(lidar_in_base[2])
    x, y = r * np.cos(a), r * np.sin(a)
    return np.stack([lidar_in_base[0] + c * x - s * y, lidar_in_base[1] + s * x + c * y], axis=1)


def pixel_bearing(u: float, fx: float, cx: float, cam_yaw_in_base: float = 0.0) -> float:
    """Piksel sütununun base çerçevesindeki yatay yönü (rad, sola pozitif)."""
    return cam_yaw_in_base - math.atan2(u - cx, fx)


def locate_from_scan(pts_base: np.ndarray, cam_in_base: Pose2D, bearing_left: float,
                     bearing_right: float, gap: float = 0.3, max_extent: float = 2.2,
                     min_points: int = 2
                     ) -> Optional[Tuple[float, float, int, np.ndarray]]:
    """
    Kutunun [bearing_right, bearing_left] yatay aralığına düşen tarama noktalarından konum.

    Aralıktaki en yakın noktadan başlanır ve küme, bir üyesine `gap` içinde olan noktalarla
    büyütülür (başlangıç noktasından en çok `max_extent` uzağa; arka plandaki duvar dışarıda
    kalır). Konum, kümenin sınır kutusunun ortasıdır: gövdeye eğik bakışta yakın
    uç yerine görünen gövdenin ortası. Döndürür: (x, y, nokta sayısı, küme noktaları Nx2), base
    çerçevesinde.
    """
    if pts_base.size == 0:
        return None
    dx, dy = pts_base[:, 0] - cam_in_base[0], pts_base[:, 1] - cam_in_base[1]
    mid = 0.5 * (bearing_left + bearing_right)
    half = 0.5 * abs(wrap(bearing_left - bearing_right))
    rel = (np.arctan2(dy, dx) - mid + np.pi) % (2 * np.pi) - np.pi
    sel = np.abs(rel) <= half
    if sel.sum() < min_points:
        return None
    p, d = pts_base[sel], np.hypot(dx, dy)[sel]
    while True:   # tek başına duran yakın noktalar (gürültü) atlanır, sıradaki en yakına geçilir
        seed = int(np.argmin(d))
        member = np.zeros(len(p), dtype=bool)
        member[seed] = True
        frontier = [seed]
        while frontier:
            i = frontier.pop()
            near = (~member) & (np.hypot(p[:, 0] - p[i, 0], p[:, 1] - p[i, 1]) <= gap) & \
                (np.hypot(p[:, 0] - p[seed, 0], p[:, 1] - p[seed, 1]) <= max_extent)
            member |= near
            frontier.extend(np.flatnonzero(near).tolist())
        if member.sum() >= min_points:
            break
        p, d = p[~member], d[~member]
        if len(p) < min_points:
            return None
    cl = p[member]
    c = 0.5 * (cl.min(axis=0) + cl.max(axis=0))
    return float(c[0]), float(c[1]), len(cl), cl


def keepout_points(discs: Sequence[Tuple[float, float, float]],
                   robot_xy: Optional[Tuple[float, float]], robot_clear: float = 0.35,
                   step: float = 0.05) -> np.ndarray:
    """
    Yasak bölge noktaları (Nx2): (x, y, yarıçap) dairelerinin içi `step` ızgarasıyla doldurulur.

    Robotun `robot_clear` yakınındaki noktalar atlanır (robot bölgenin içindeyse kendi hücresi
    engel sayılıp planlayıcıyı kilitlemesin).
    """
    out: List[np.ndarray] = []
    for x, y, r in discs:
        n = int(math.ceil(r / step))
        ax = np.arange(-n, n + 1) * step
        gx, gy = np.meshgrid(ax, ax)
        keep = gx * gx + gy * gy <= r * r
        out.append(np.stack([x + gx[keep], y + gy[keep]], axis=1))
    if not out:
        return np.zeros((0, 2))
    pts = np.unique(np.round(np.concatenate(out) / step).astype(np.int64), axis=0) * step
    if robot_xy is not None:
        pts = pts[np.hypot(pts[:, 0] - robot_xy[0], pts[:, 1] - robot_xy[1]) > robot_clear]
    return pts


def ground_ray(u: float, v: float, fx: float, fy: float, cx: float, cy: float,
               cam_in_base: Pose2D, cam_z: float, max_range: float = 8.0
               ) -> Optional[Tuple[float, float]]:
    """Piksel (u, v) ışınının z=0 zemin düzlemiyle kesişimi (base); ufkun üstündeyse None."""
    down = (v - cy) / fy          # optik y aşağı: ileri 1 m'de düşüş (m)
    if down <= 1e-3:
        return None
    fwd = cam_z / down
    if fwd > max_range:
        return None
    left = -(u - cx) / fx * fwd
    return to_frame(cam_in_base, fwd, left)


class Grid:
    """nav_msgs/OccupancyGrid'in ROS'suz karşılığı (dönüşsüz köken varsayılır)."""

    def __init__(self, data: np.ndarray, resolution: float, origin_x: float,
                 origin_y: float) -> None:
        self.data, self.res, self.ox, self.oy = data, resolution, origin_x, origin_y

    def cost(self, x: float, y: float) -> int:
        """Hücre maliyeti 0..100; harita dışı ya da bilinmeyen = 100."""
        j, i = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        if i < 0 or j < 0 or i >= self.data.shape[0] or j >= self.data.shape[1]:
            return 100
        c = int(self.data[i, j])
        return 100 if c < 0 else c


def plan_viewpoints(grid: Grid, px: float, py: float, radius: float, count: int,
                    start_xy: Tuple[float, float], max_cost: int = 60,
                    lethal_cost: int = 99, clear_radius: float = 0.7,
                    min_separation: float = math.radians(50.0),
                    step: float = math.radians(15.0)) -> List[Pose2D]:
    """
    Kişinin (px, py) çevresinde `radius` uzaklıkta en çok `count` bakış noktası.

    Her noktanın yönü kişiye dönüktür.

    Aday kabul koşulu: hücre maliyeti ≤ max_cost ve adaydan kişiye doğru, kişinin `clear_radius`
    yakınına kadar yolda öldürücü hücre yok (kişinin kendisi costmap'te engeldir). İlk nokta
    robota en yakın geçerli adaydır; sonrakiler seçilenlere açısal olarak en uzak olanlardır
    (en az `min_separation`).
    """
    cands: List[Tuple[float, Pose2D]] = []
    n = int(round(2 * math.pi / step))
    for k in range(n):
        a = k * step
        x, y = px + radius * math.cos(a), py + radius * math.sin(a)
        if grid.cost(x, y) > max_cost:
            continue
        clear = True
        t = 0.0
        while t < radius - clear_radius:
            t += grid.res
            if grid.cost(x - t * math.cos(a), y - t * math.sin(a)) >= lethal_cost:
                clear = False
                break
        if clear:
            cands.append((a, (x, y, wrap(a + math.pi))))
    if not cands:
        return []
    first = min(cands, key=lambda c: math.hypot(c[1][0] - start_xy[0], c[1][1] - start_xy[1]))
    chosen = [first]
    while len(chosen) < count:
        best, best_sep = None, min_separation
        for c in cands:
            sep = min(abs(wrap(c[0] - q[0])) for q in chosen)
            if sep >= best_sep:
                best, best_sep = c, sep
        if best is None:
            break
        chosen.append(best)
    return [c[1] for c in chosen]


def check_view(pts_base: np.ndarray, robot_in_map: Pose2D, cam_in_base: Pose2D,
               lidar_in_base: Pose2D, px: float, py: float, hfov: float,
               max_range: float, min_range: float = 0.5, half_width: float = 0.35,
               depth_tol: float = 0.45, fov_margin: float = math.radians(6.0)) -> dict:
    """
    Kişinin kayıtlı konumunun (px, py; map) şu an gerçekten görülebildiğini doğrular.

    in_fov    : konum kamera yatay görüş alanında (kenar payı ile) ve [min_range, max_range] içinde
    blocked   : LiDAR'da konumdan belirgin biçimde ÖNDE bir engel var (görüş kapalı)
    occupied  : LiDAR konumun bulunduğu yerde hâlâ bir cisim görüyor
    clear     : in_fov ve engel yok ve konum boş → "kişi yok" kararı verilebilir
    """
    # konum → base
    c, s = math.cos(robot_in_map[2]), math.sin(robot_in_map[2])
    dx, dy = px - robot_in_map[0], py - robot_in_map[1]
    bx, by = c * dx + s * dy, -s * dx + c * dy
    cam_d = math.hypot(bx - cam_in_base[0], by - cam_in_base[1])
    cam_b = wrap(math.atan2(by - cam_in_base[1], bx - cam_in_base[0]) - cam_in_base[2])
    in_fov = abs(cam_b) <= hfov / 2.0 - fov_margin and min_range <= cam_d <= max_range
    out = {'in_fov': bool(in_fov), 'cam_dist_m': round(cam_d, 2),
           'cam_bearing_deg': round(math.degrees(cam_b), 1), 'beams': 0, 'blocked': False,
           'occupied': False, 'clear': False}
    if pts_base.size == 0:
        return out
    # LiDAR'dan konuma doğru bakan ışınlar
    lx, ly = pts_base[:, 0] - lidar_in_base[0], pts_base[:, 1] - lidar_in_base[1]
    tx, ty = bx - lidar_in_base[0], by - lidar_in_base[1]
    td = math.hypot(tx, ty)
    if td < 1e-3:
        return out
    bearing = np.arctan2(ly, lx) - math.atan2(ty, tx)
    bearing = (bearing + np.pi) % (2 * np.pi) - np.pi
    sel = np.abs(bearing) <= math.atan2(half_width, td)
    d = np.hypot(lx, ly)[sel]
    out['beams'] = int(d.size)
    if d.size < 2:
        return out
    front = float((d < td - depth_tol).mean())
    at = float((np.abs(d - td) <= depth_tol).mean())
    out.update({'frac_front': round(front, 2), 'frac_at': round(at, 2),
                'blocked': front > 0.2, 'occupied': at > 0.2})
    out['clear'] = bool(in_fov and not out['blocked'] and not out['occupied'])
    return out
