"""
coverage_monitor: robotun GERÇEK yolundan, simüle kamera görüşüyle kapsama yüzdesini ölçer.

Her `rate` Hz'de (sim saati) TF map→camera_frame okunur; kamera konumundan, yaw yönünde FOV ve
menzil içinde ışın izlemeyle görülen alan işaretlenir (patrol_planner ile aynı model:
coverage_model.Visibility). Harita ve robot parametreleri planlayıcıyla aynı dosyalardan okunur.

Rapor (her `report` sim-s ve bitişte): toplam / dar / geniş bölge kapsaması, robot merkezinin
içinden geçtiği koridor segmentleri (patrol_planner.corridor_segments; segment pikseline ≤ 0.3 m),
kat edilen yol. SIGINT/SIGTERM (arka planda da) ya da `duration` dolunca "SON ÖZET" basar;
`png` verilmişse kapsama + yol görselini yazar.

Parametreler: map_yaml, robot_params, rate (2.0), duration (0 = sınırsız, sim-s), report (120),
  png (''), frame_id (map).
"""

import math
import signal
from typing import List, Optional, Tuple

from fsb_patrol.coverage_model import analyze, load_map, load_robot_params, Visibility
from fsb_patrol.patrol_planner import make_candidates
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy import ndimage
from tf2_ros import Buffer, TransformListener


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CoverageMonitor(Node):

    def __init__(self) -> None:
        super().__init__('fsb_coverage_monitor')
        for name, default in (('map_yaml', ''), ('robot_params', ''), ('rate', 2.0),
                              ('duration', 0.0), ('report', 120.0), ('png', ''),
                              ('frame_id', 'map')):
            self.declare_parameter(name, default)
        gp = self.get_parameter
        self.gm = load_map(gp('map_yaml').value)
        self.p = load_robot_params(gp('robot_params').value)
        self.ma = analyze(self.gm, self.p)
        self.vis = Visibility(self.ma)
        _, _, self.segs = make_candidates(self.ma)
        # Segment "girildi" testi: segment pikselinden ≤ 0.3 m
        lab = np.full(self.gm.shape, -1, dtype=np.int32)
        for k, px in enumerate(self.segs):
            lab[px[:, 0], px[:, 1]] = k
        near = int(round(0.3 / self.gm.resolution))
        self.seg_near = ndimage.grey_dilation(lab + 1, size=(2 * near + 1, 2 * near + 1)) - 1
        self.seg_entered = set()
        self.covered = np.zeros(self.ma.n_blocks, dtype=bool)
        self.path: List[Tuple[float, float]] = []
        self.path_len = 0.0
        self.samples = 0
        self.frame_id: str = gp('frame_id').value
        self.cam: str = self.p.camera_frame
        self.duration = float(gp('duration').value)
        self.report = float(gp('report').value)
        self.png: str = gp('png').value
        self.t0: Optional[float] = None
        self.next_report = 0.0
        self.done = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / float(gp('rate').value), self._tick)
        self.get_logger().info(
            f'Harita {self.gm.source}: kapsama evreni {self.ma.n_blocks} blok '
            f'({self.ma.universe_free.sum() * self.gm.resolution ** 2:.1f} m², dar %'
            f'{100 * self.ma.block_narrow.mean():.0f}), {len(self.segs)} koridor segmenti; '
            f'kamera={self.cam}, FOV={math.degrees(self.p.camera_hfov):.1f}°, '
            f'menzil={self.p.sensor_range:.1f} m')

    def _tick(self) -> None:
        try:
            tr = self.tf_buffer.lookup_transform(self.frame_id, self.cam, Time(),
                                                 timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001 - TF henüz yok
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
            self.next_report = t + self.report
        x, y = tr.transform.translation.x, tr.transform.translation.y
        yaw = yaw_of(tr.transform.rotation)
        r, c = self.gm.to_grid(x, y)
        r, c = int(r), int(c)
        h, w = self.gm.shape
        if 0 <= r < h and 0 <= c < w:
            self.covered[self.vis.seen_from_pose(r, c, yaw)] = True
            k = int(self.seg_near[r, c])
            if k >= 0:
                self.seg_entered.add(k)
        if self.path:
            self.path_len += math.hypot(x - self.path[-1][0], y - self.path[-1][1])
        self.path.append((x, y))
        self.samples += 1
        if t >= self.next_report:
            self.next_report += self.report
            self.summary('ARA ÖZET')
        if self.duration and t - self.t0 >= self.duration:
            self.done = True

    def stats(self) -> dict:
        nb = self.ma.block_narrow
        return {'total': float(self.covered.mean()),
                'narrow': float(self.covered[nb].mean()) if nb.any() else 1.0,
                'wide': float(self.covered[~nb].mean()) if (~nb).any() else 1.0,
                'segments': len(self.seg_entered), 'n_segments': len(self.segs)}

    def summary(self, tag: str) -> None:
        s = self.stats()
        el = (self.get_clock().now().nanoseconds * 1e-9 - self.t0) if self.t0 else 0.0
        self.get_logger().info(
            f'--- {tag}: {el:.0f} sim-s, {self.samples} örnek, yol {self.path_len:.1f} m --- '
            f'KAPSAMA toplam %{100 * s["total"]:.2f}  dar %{100 * s["narrow"]:.2f}  '
            f'geniş %{100 * s["wide"]:.2f}  | koridor segmenti girildi '
            f'{s["segments"]}/{s["n_segments"]}')

    def write_png(self) -> None:
        if not self.png:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        gm, ma = self.gm, self.ma
        h, w = gm.shape
        img = np.full((h, w, 3), 0.55)
        img[ma.free] = (1.0, 1.0, 1.0)
        cov = np.zeros((h, w), dtype=bool)
        m = ma.block_id >= 0
        cov[m] = self.covered[ma.block_id[m]]
        img[ma.universe_free & cov] = (0.8, 0.93, 0.8)
        img[ma.universe_free & ~cov] = (1.0, 0.7, 0.7)
        img[gm.grid == 100] = (0.1, 0.1, 0.1)
        for px in self.segs:
            img[px[:, 0], px[:, 1]] = (0.55, 0.2, 0.75)
        ext = [gm.origin[0], gm.origin[0] + w * gm.resolution,
               gm.origin[1], gm.origin[1] + h * gm.resolution]
        fig, ax = plt.subplots(figsize=(13, 9), dpi=120)
        ax.imshow(img, extent=ext, interpolation='nearest')
        if self.path:
            pa = np.array(self.path)
            ax.plot(pa[:, 0], pa[:, 1], '-', color='#2563eb', lw=0.8)
        s = self.stats()
        ax.set_title(f'Gerçek yoldan kapsama %{100 * s["total"]:.1f} '
                     f'(dar %{100 * s["narrow"]:.1f}) · '
                     f'yol {self.path_len:.0f} m · koridor {s["segments"]}/{s["n_segments"]} · '
                     'yeşil: görüldü, pembe: görülmedi')
        rr, cc = np.nonzero(ma.universe_free)
        fx, fy = gm.to_world(rr, cc)
        ax.set_xlim(float(fx.min()) - 0.5, float(fx.max()) + 0.5)
        ax.set_ylim(float(fy.min()) - 0.5, float(fy.max()) + 0.5)
        ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(self.png)
        plt.close(fig)
        self.get_logger().info(f'görsel: {self.png}')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CoverageMonitor()
    stop = {'v': False}
    # Arka planda (&) başlatılan süreçler SIGINT'i yok sayabilir; işleyiciyi açıkça kur.
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('v', True))
    while rclpy.ok() and not stop['v'] and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.summary('SON ÖZET')
    node.write_png()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
