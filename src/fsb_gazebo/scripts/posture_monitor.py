#!/usr/bin/env python3
"""
Duruş/alarm sayacı: yatan insan algılama doğruluğu ve yanlış alarm ölçümü.

İzler: /perception/posture (yolo_node'un kare başına JSON çıktısı) ve /security/alerts.
Sayar: çıkarım karesi, insan görülen kare, duruşa göre kare (lying / upright / unknown),
"yatan" kararını veren ipucu (torso / aspect), alarm sayısı ve sim zamanları.
Negatif dünyada (ayakta insan + kutu) lying kare ve alarm sayısı = yanlış alarm ölçüsü.

Kullanım:
  ros2 run fsb_gazebo posture_monitor.py [--duration SİM_S] [--json çıktı.json]
  (Ctrl-C ya da süre dolunca özet basar.)
"""

import argparse
import json
import signal
from typing import Any, Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from vision_msgs.msg import Detection2D


class Monitor(Node):

    def __init__(self) -> None:
        super().__init__('fsb_posture_monitor',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.frames = 0
        self.person_frames = 0
        self.by_posture: Dict[str, int] = {'lying': 0, 'upright': 0, 'unknown': 0}
        self.by_cue: Dict[str, int] = {'torso': 0, 'aspect': 0}
        self.lying_scores: List[float] = []
        self.upright_scores: List[float] = []
        self.alerts: List[Dict[str, Any]] = []
        self.create_subscription(String, '/perception/posture', self._on_posture, 50)
        self.create_subscription(Detection2D, '/security/alerts', self._on_alert, 20)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_posture(self, m: String) -> None:
        people = json.loads(m.data).get('people', [])
        self.frames += 1
        if not people:
            return
        self.person_frames += 1
        # Kare, içindeki en "ciddi" duruşla sayılır: lying > upright > unknown
        kinds = {p['posture'] for p in people}
        kind = 'lying' if 'lying' in kinds else 'upright' if 'upright' in kinds else 'unknown'
        self.by_posture[kind] += 1
        best = max((p for p in people if p['posture'] == kind), key=lambda p: p['score'])
        if kind == 'lying':
            self.by_cue[best['cue']] = self.by_cue.get(best['cue'], 0) + 1
            self.lying_scores.append(best['score'])
        elif kind == 'upright':
            self.upright_scores.append(best['score'])

    def _on_alert(self, m: Detection2D) -> None:
        self.alerts.append({'sim_time': round(self.now(), 2),
                            'score': round(m.results[0].hypothesis.score, 3)})

    def summary(self) -> Dict[str, Any]:
        def stat(v: List[float]) -> Dict[str, float]:
            if not v:
                return {}
            s = sorted(v)
            return {'min': round(s[0], 3), 'median': round(s[len(s) // 2], 3),
                    'max': round(s[-1], 3)}
        return {'frames': self.frames, 'person_frames': self.person_frames,
                'frames_by_posture': self.by_posture, 'lying_frames_by_cue': self.by_cue,
                'lying_score': stat(self.lying_scores),
                'upright_score': stat(self.upright_scores),
                'alerts': len(self.alerts), 'alert_list': self.alerts}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=0.0, help='sim-s; 0 = Ctrl-C bekle')
    ap.add_argument('--json', default='', help='özetin yazılacağı dosya')
    args = ap.parse_args()

    def stop(*_: Any) -> None:
        raise KeyboardInterrupt

    # arka planda (SIGINT yok sayılırken) ya da SIGTERM ile durdurulunca da özet yazılsın
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = Monitor()
    start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.now()
            if start is None and now > 0.0:
                start = now
            if args.duration > 0.0 and start is not None and now - start >= args.duration:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    s = node.summary()
    text = json.dumps(s, indent=2, ensure_ascii=False)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    node.get_logger().info('Özet:\n' + text) if rclpy.ok() else None
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
