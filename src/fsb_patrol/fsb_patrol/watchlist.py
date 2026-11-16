"""
Olay listesi (watchlist): yerde yatan kişi kayıtları ve kalıcı saklama.

Dosyalar (dizin: patrol_executor `alert_log_dir`):
  watchlist.yaml  güncel liste (her değişiklikte atomik yazılır)
  events.jsonl    zaman sıralı olay günlüğü (added / observation / revisit / resolved ...)
  wl<id>_*.jpg    kayıt ve revisit kareleri

Kayıt alanları: id, x, y (map çerçevesi, m), status (active | resolved), first_seen_wall,
first_seen_sim, last_seen_sim, confirmations (revisit'te "hâlâ yerde" doğrulama sayısı),
notifications (bildirim sayısı: ilk tespit + her doğrulama), images, observations (açı başına
görüntü + duruş kaydı), revisits (her revisit'in sonucu), next_revisit_sim, resolved_wall,
extent (gövdenin LiDAR'da görülen noktaları, map; aynı kişiyi birleştirmede kullanılır).

Zamanlar ROS zamanıdır (sim'de sim-s). Önceki oturumdan yüklenen etkin kayıtların revisit zamanı
yükleme anına göre yeniden kurulur (sim zamanı her açılışta sıfırdan başlar).
"""

from dataclasses import asdict, dataclass, field
import json
import math
import os
import tempfile
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence

import yaml


@dataclass
class WatchEntry:
    id: int  # noqa: A003 - kayıt alanı adı
    x: float
    y: float
    status: str = 'active'
    first_seen_wall: str = ''
    first_seen_sim: float = 0.0
    last_seen_sim: float = 0.0
    confirmations: int = 0
    notifications: int = 0
    images: List[str] = field(default_factory=list)
    observations: List[Dict[str, Any]] = field(default_factory=list)
    revisits: List[Dict[str, Any]] = field(default_factory=list)
    next_revisit_sim: float = math.inf
    resolved_wall: str = ''
    extent: List[List[float]] = field(default_factory=list)


def wall_now() -> str:
    return time.strftime('%Y-%m-%dT%H:%M:%S')


class Watchlist:

    def __init__(self, directory: str, merge_radius: float = 1.0) -> None:
        self.dir = os.path.expanduser(directory)
        self.merge_radius = merge_radius
        self.entries: List[WatchEntry] = []
        self.path = os.path.join(self.dir, 'watchlist.yaml')
        self.events_path = os.path.join(self.dir, 'events.jsonl')

    # ------------------------------------------------------------------ saklama
    def load(self, now_sim: float, revisit_interval: float) -> int:
        """Varsa önceki listeyi yükler; etkin kayıt sayısını döndürür."""
        if not os.path.isfile(self.path):
            return 0
        with open(self.path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        known = set(WatchEntry.__dataclass_fields__)
        for raw in doc.get('watchlist', []):
            e = WatchEntry(**{k: v for k, v in raw.items() if k in known})
            if e.status == 'active':
                e.next_revisit_sim = now_sim + revisit_interval
            self.entries.append(e)
        return len(self.active())

    def save(self) -> None:
        os.makedirs(self.dir, exist_ok=True)
        doc = {'updated_wall': wall_now(), 'merge_radius_m': self.merge_radius,
               'watchlist': [self._plain(e) for e in self.entries]}
        fd, tmp = tempfile.mkstemp(dir=self.dir, suffix='.tmp')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, self.path)

    @staticmethod
    def _plain(e: WatchEntry) -> Dict[str, Any]:
        d = asdict(e)
        if math.isinf(d['next_revisit_sim']):
            d['next_revisit_sim'] = None
        return d

    def log_event(self, event: str, **data: Any) -> None:
        os.makedirs(self.dir, exist_ok=True)
        rec = {'wall_time': wall_now(), 'event': event}
        rec.update(data)
        with open(self.events_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    # ------------------------------------------------------------------ sorgu / değişiklik
    def active(self) -> List[WatchEntry]:
        return [e for e in self.entries if e.status == 'active']

    def find(self, x: float, y: float, radius: Optional[float] = None) -> Optional[WatchEntry]:
        """
        (x, y)'ye birleştirme yarıçapı içindeki en yakın ETKİN kayıt.

        Uzaklık kaydın merkezine ya da bilinen gövde noktalarına (extent) en kısa uzaklıktır:
        1.7 m'lik gövdenin iki ucu ayrı kişi sayılmaz.
        """
        r = self.merge_radius if radius is None else radius
        best, best_d = None, r
        for e in self.active():
            d = self.distance(e, x, y)
            if d <= best_d:
                best, best_d = e, d
        return best

    @staticmethod
    def ray_distance(e: WatchEntry, ox: float, oy: float, x: float, y: float) -> float:
        """Kaydın, (ox, oy)'den (x, y)'ye doğru uzanan yarı doğruya en kısa uzaklığı."""
        ux, uy = x - ox, y - oy
        n = math.hypot(ux, uy)
        if n < 1e-6:
            return math.hypot(e.x - ox, e.y - oy)
        ux, uy = ux / n, uy / n
        best = math.inf
        for px, py in [(e.x, e.y)] + list(e.extent):
            t = max(0.0, (px - ox) * ux + (py - oy) * uy)
            best = min(best, math.hypot(px - ox - t * ux, py - oy - t * uy))
        return best

    @staticmethod
    def distance(e: WatchEntry, x: float, y: float) -> float:
        """(x, y)'nin kayda uzaklığı: merkez ve gövde noktalarının en yakını."""
        return min(math.hypot(px - x, py - y) for px, py in [(e.x, e.y)] + list(e.extent))

    @staticmethod
    def set_extent(e: WatchEntry, points: Iterable[Sequence[float]], cell: float = 0.15,
                   cap: int = 120) -> None:
        """Gövde noktalarını `cell` ızgarasında seyrelterek saklar (en çok `cap` nokta)."""
        seen = {}
        for px, py in list(e.extent) + [list(p) for p in points]:
            seen.setdefault((round(px / cell), round(py / cell)),
                            [round(float(px), 2), round(float(py), 2)])
        e.extent = list(seen.values())[:cap]

    def add(self, x: float, y: float, now_sim: float) -> WatchEntry:
        e = WatchEntry(id=1 + max([q.id for q in self.entries], default=0),
                       x=round(float(x), 3), y=round(float(y), 3), first_seen_wall=wall_now(),
                       first_seen_sim=round(now_sim, 3), last_seen_sim=round(now_sim, 3),
                       notifications=1)
        self.entries.append(e)
        return e

    def due(self, now_sim: float) -> Optional[WatchEntry]:
        """Revisit zamanı gelmiş etkin kayıtlardan en eskisi."""
        cand = [e for e in self.active() if e.next_revisit_sim is not None
                and e.next_revisit_sim <= now_sim]
        return min(cand, key=lambda e: e.next_revisit_sim) if cand else None

    def resolve(self, e: WatchEntry) -> None:
        e.status = 'resolved'
        e.resolved_wall = wall_now()
        e.next_revisit_sim = math.inf
