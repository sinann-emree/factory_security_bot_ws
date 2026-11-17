"""Olay geometrisi ve olay listesi birim testleri (ROS gerekmez)."""

import math

from fsb_patrol import incident_geometry as geo
from fsb_patrol.watchlist import Watchlist
import numpy as np

LIDAR = (0.0, 0.0, math.pi)   # 180° dönük montaj
CAM = (0.06, 0.0, 0.0)


def scan_with(obstacles, n=360, far=10.0):
    """Tarama üretir: base'deki (x, y, yarıçap) dairelerinden 360 ışın (LiDAR çerçevesi)."""
    ranges = np.full(n, far)
    for k in range(n):
        a = -math.pi + 2 * math.pi * k / n + LIDAR[2]       # ışının base'deki yönü
        for ox, oy, r in obstacles:
            b = ox * math.cos(a) + oy * math.sin(a)
            d2 = ox * ox + oy * oy - b * b
            if b > 0 and d2 <= r * r:
                ranges[k] = min(ranges[k], b - math.sqrt(r * r - d2))
    return ranges, -math.pi, 2 * math.pi / n


def pts_of(obstacles, keep_far=False):
    r, a0, da = scan_with(obstacles)
    return geo.scan_points(r, a0, da, LIDAR, 0.15, 9.0, keep_far=keep_far)


def test_scan_points_respect_lidar_mounting():
    pts = pts_of([(3.0, 0.0, 0.3)])
    front = pts[np.abs(pts[:, 1]) < 0.2]
    assert len(front) > 0 and abs(front[:, 0].min() - 2.7) < 0.05   # engel önde, arkada değil


def test_locate_from_scan_ignores_background():
    pts = pts_of([(3.0, 1.0, 0.3), (8.0, 2.6, 1.0)])     # kişi + arkasında duvar parçası
    fx, cx = 530.0, 320.0
    u = cx - fx * math.tan(math.atan2(1.0, 3.0 - CAM[0]))   # kişinin piksel sütunu
    hit = geo.locate_from_scan(pts, CAM, geo.pixel_bearing(u - 40, fx, cx),
                               geo.pixel_bearing(u + 40, fx, cx))
    assert hit is not None
    assert math.hypot(hit[0] - 3.0, hit[1] - 1.0) < 0.4


def test_locate_centres_on_visible_body_not_near_end():
    # eğik duran 1.6 m'lik gövde: (2.0, -0.6) → (3.2, 0.5); arkada duvar
    body = [(2.0 + 1.2 * k / 16, -0.6 + 1.1 * k / 16) for k in range(17)]
    pts = np.array(body + [(6.0, y / 10.0) for y in range(-20, 21)])
    hit = geo.locate_from_scan(pts, CAM, 0.5, -0.5)
    assert hit is not None and hit[2] == 17 and len(hit[3]) == 17
    assert math.hypot(hit[0] - 2.6, hit[1] + 0.05) < 0.05


def test_watchlist_merges_along_body_extent(tmp_path):
    wl = Watchlist(str(tmp_path), merge_radius=1.0)
    e = wl.add(0.0, 0.0, 10.0)
    assert wl.find(1.4, 0.0) is None                       # yalnızca merkez: 1.4 m uzak
    wl.set_extent(e, [(x / 10.0, 0.0) for x in range(-8, 9)])
    assert wl.find(1.4, 0.0) is e                          # gövdenin ucuna 0.6 m
    assert wl.find(2.0, 0.0) is None
    # yön üzerinde: (-5, 0.3)'ten bakılırken menzili yanlış (kısa) kestirilen aynı kişi
    assert wl.ray_distance(e, -5.0, 0.3, -2.5, 0.2) < 1.0
    assert wl.ray_distance(e, -5.0, 0.3, -5.0, 3.0) > 1.0      # başka yöne bakış


def test_locate_skips_isolated_near_point():
    pts = np.array([(1.0, 0.0)] + [(3.0, y / 10.0) for y in range(-4, 5)])
    hit = geo.locate_from_scan(pts, CAM, 0.3, -0.3)
    assert hit is not None and abs(hit[0] - 3.0) < 1e-6 and hit[2] == 9


def test_keepout_points_fill_disc_and_spare_robot():
    pts = geo.keepout_points([(1.0, 2.0, 1.0)], (1.0, 1.2))
    d = np.hypot(pts[:, 0] - 1.0, pts[:, 1] - 2.0)
    assert d.max() <= 1.0 + 1e-6 and len(pts) > 1000
    assert np.hypot(pts[:, 0] - 1.0, pts[:, 1] - 1.2).min() > 0.35
    assert geo.keepout_points([], None).shape == (0, 2)


def test_locate_returns_none_without_points():
    assert geo.locate_from_scan(np.zeros((0, 2)), CAM, 0.1, -0.1) is None


def test_ground_ray_horizon_and_range():
    assert geo.ground_ray(320, 230, 530, 530, 320, 240, CAM, 0.047) is None      # ufkun üstü
    x, y = geo.ground_ray(320, 240 + 53, 530, 530, 320, 240, CAM, 0.3)
    assert abs(x - (0.06 + 3.0)) < 1e-6 and abs(y) < 1e-6


def empty_grid(size=200, res=0.05):
    return geo.Grid(np.zeros((size, size), dtype=np.int16), res, -5.0, -5.0)


def test_viewpoints_face_person_and_spread():
    g = empty_grid()
    g.data[95:105, 95:105] = 100            # kişinin kendisi costmap'te engel
    vps = geo.plan_viewpoints(g, 0.0, 0.0, 1.75, 3, (-3.0, 0.0))
    assert len(vps) == 3
    for x, y, yaw in vps:
        assert abs(math.hypot(x, y) - 1.75) < 1e-6
        assert abs(geo.wrap(yaw - math.atan2(-y, -x))) < 1e-6      # kişiye bakıyor
    assert math.hypot(vps[0][0] + 1.75, vps[0][1]) < 0.5           # ilki robota en yakın
    angs = [math.atan2(y, x) for x, y, _ in vps]
    assert min(abs(geo.wrap(a - b)) for i, a in enumerate(angs) for b in angs[i + 1:]) \
        >= math.radians(50) - 1e-6


def test_viewpoints_skip_blocked_side():
    g = empty_grid()
    g.data[:, 110:] = 100                   # doğu tarafı duvar
    vps = geo.plan_viewpoints(g, 0.0, 0.0, 1.75, 3, (3.0, 0.0))
    assert vps and all(x < 0.6 for x, _, _ in vps)


def view(obstacles, robot=(0.0, 0.0, 0.0), target=(2.5, 0.0)):
    return geo.check_view(pts_of(obstacles, keep_far=True), robot, CAM, LIDAR, target[0],
                          target[1], 1.085, 4.0)


def test_view_clear_when_person_gone():
    v = view([])
    assert v['in_fov'] and not v['blocked'] and not v['occupied'] and v['clear']


def test_view_not_clear_when_something_still_there():
    v = view([(2.5, 0.0, 0.3)])
    assert v['occupied'] and not v['clear']


def test_view_not_clear_when_occluded():
    v = view([(1.2, 0.0, 0.3)])
    assert v['blocked'] and not v['clear']


def test_view_not_clear_outside_fov_or_range():
    assert not view([], target=(0.0, 2.5))['clear']        # yanda: kamera görmüyor
    assert not view([], target=(6.0, 0.0))['clear']        # çok uzak
    assert not view([], robot=(0.0, 0.0, math.pi))['clear']  # robot arkasını dönmüş


def test_watchlist_merge_and_persistence(tmp_path):
    wl = Watchlist(str(tmp_path), merge_radius=1.0)
    a = wl.add(1.0, 1.0, now_sim=10.0)
    assert wl.find(1.6, 1.5) is a           # 0.78 m: aynı kişi
    assert wl.find(2.5, 1.0) is None        # 1.5 m: başka kişi
    b = wl.add(5.0, 5.0, now_sim=20.0)
    assert (a.id, b.id) == (1, 2) and a.notifications == 1
    a.next_revisit_sim = 100.0
    assert wl.due(50.0) is None and wl.due(100.0) is a
    wl.resolve(a)
    assert wl.find(1.0, 1.0) is None and wl.due(1e9) is None   # çözülen kayıt eşleşmez
    wl.log_event('resolved', id=a.id)
    wl.save()
    again = Watchlist(str(tmp_path), merge_radius=1.0)
    assert again.load(now_sim=0.0, revisit_interval=300.0) == 1
    assert [e.status for e in again.entries] == ['resolved', 'active']
    assert again.entries[1].next_revisit_sim == 300.0          # revisit zamanı yeniden kuruldu
