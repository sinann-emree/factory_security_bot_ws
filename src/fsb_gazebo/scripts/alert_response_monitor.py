#!/usr/bin/env python3
"""
Algı + alarm tepkisi ölçümü (YOLO entegrasyonu çıkış kriteri).

İzler: /perception/detections (güven dağılımı), /security/alerts (alarm zamanları),
/odometry/filtered (robot hızı). Her "ele alınan" alarm için (alarmdan sonra robot durduysa):
alarm anındaki hız, durma gecikmesi, durma süresi, yeniden hareket. Yok sayılan alarmlar
(patrol_executor bekleme süresi içinde) robotu durdurmaz; ayrı sayılır.

Kullanım:
  ros2 run fsb_gazebo alert_response_monitor.py [--duration SİM_S] [--json çıktı.json]
  (Ctrl-C ya da süre dolunca özet basar.)
"""

import argparse
import json
from typing import Any, Dict, List, Optional

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from vision_msgs.msg import Detection2D, Detection2DArray

V_STOP, W_STOP = 0.01, 0.02   # m/s, rad/s: "durdu" eşiği
STOP_WINDOW = 3.0             # sim-s: alarmdan sonra bu sürede durmadıysa "yok sayıldı"
MIN_STOP = 1.0                # sim-s: bundan kısa duruş alarm tepkisi sayılmaz


class Monitor(Node):

    def __init__(self, target: str) -> None:
        super().__init__('fsb_alert_response_monitor',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.target = target
        self.frames = 0
        self.scores: List[float] = []          # hedef görülen karelerde en yüksek güven
        self.alerts: List[Dict[str, Any]] = []
        self.speed = (0.0, 0.0)
        self.open: Optional[Dict[str, Any]] = None
        self.create_subscription(Detection2DArray, '/perception/detections', self._on_det, 20)
        self.create_subscription(Detection2D, '/security/alerts', self._on_alert, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 20)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_det(self, m: Detection2DArray) -> None:
        self.frames += 1
        s = [d.results[0].hypothesis.score for d in m.detections
             if d.results and d.results[0].hypothesis.class_id == self.target]
        if s:
            self.scores.append(max(s))

    def _on_alert(self, m: Detection2D) -> None:
        rec = {'sim_time': round(self.now(), 2), 'score': round(m.results[0].hypothesis.score, 3),
               'v_at_alert': round(self.speed[0], 3), 'w_at_alert': round(self.speed[1], 3),
               'handled': False}
        self.alerts.append(rec)
        if self.open is None:
            self.open = rec

    def _on_odom(self, m: Odometry) -> None:
        v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
        self.speed = (v, w)
        o = self.open
        if o is None:
            return
        t = self.now()
        still = abs(v) < V_STOP and abs(w) < W_STOP
        if 'stopped_at' not in o:
            if still:
                o['stopped_at'] = t
                o['stop_latency_s'] = round(t - o['sim_time'], 2)
                o['handled'] = True
            elif t - o['sim_time'] > STOP_WINDOW:
                self.open = None        # durmadı: executor yok saydı (bekleme süresi)
        elif not still:
            dur = t - o.pop('stopped_at')
            if dur < MIN_STOP:          # anlık duraksama (ör. hedef değişimi), alarm tepkisi değil
                o['handled'] = False
                o.pop('stop_latency_s', None)
            else:
                o['stopped_for_s'] = round(dur, 2)
                o['resumed'] = True
            self.open = None

    def summary(self) -> Dict[str, Any]:
        s = np.array(self.scores) if self.scores else np.zeros(0)
        handled = [a for a in self.alerts if a['handled']]
        moving = [a for a in handled if abs(a['v_at_alert']) >= V_STOP
                  or abs(a['w_at_alert']) >= W_STOP]
        return {
            'target': self.target, 'inference_frames': self.frames,
            'frames_with_target': int(s.size),
            'score_median': round(float(np.median(s)), 3) if s.size else None,
            'score_mean': round(float(s.mean()), 3) if s.size else None,
            'score_max': round(float(s.max()), 3) if s.size else None,
            'frames_ge_0.8': int((s >= 0.8).sum()),
            'frames_ge_0.8_pct': round(100.0 * float((s >= 0.8).mean()), 1) if s.size else None,
            'alerts_published': len(self.alerts), 'alerts_handled_robot_stopped': len(handled),
            'handled_while_moving': len(moving),
            'stop_latency_s': [a.get('stop_latency_s') for a in handled],
            'stopped_for_s': [a.get('stopped_for_s') for a in handled],
            'resumed': sum(1 for a in handled if a.get('resumed')),
            'handled_alerts': handled,
        }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=0.0, help='sim-s; 0 = Ctrl-C bekle')
    ap.add_argument('--json', default='')
    ap.add_argument('--target', default='person')
    args = ap.parse_args()
    rclpy.init()
    node = Monitor(args.target)
    try:
        t0 = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration > 0 and node.now() > 0:
                t0 = node.now() if t0 is None else t0
                if node.now() - t0 >= args.duration:
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():   # kapanışta (SIGINT) geçersiz bağlam hatası: özet yine basılır
            raise
    text = json.dumps(node.summary(), indent=2, ensure_ascii=False)
    print(text, flush=True)   # noqa: T201 - rclpy kapanmış olabilir; özet stdout'a
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
