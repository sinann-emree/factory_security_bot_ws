"""
Duruş sınıflandırma: yatan / ayakta / belirsiz (ROS'tan ve torch'tan bağımsız).

Girdi: kişi kutusu (genişlik, yükseklik; piksel) ve COCO 17 anahtar noktası (x, y, güven).
İki ipucu:
  1. Gövde ekseni: omuz ortası (5, 6) → kalça ortası (11, 12) doğrusunun görüntü düşeyiyle açısı
     (0° = dik, 90° = yatay). Dört noktanın hepsi `kp_conf` üstündeyse geçerlidir.
  2. Kutu oranı = genişlik / yükseklik (anahtar noktalar güvenilmezken yedek yol).
Karar:
  gövde geçerli ve açı ≥ torso_lying_deg                      → yatan   (gövde ekseni)
  gövde geçerli ve açı ≤ torso_upright_deg ve oran < aspect_lying → ayakta
  oran ≥ aspect_lying                                          → yatan   (kutu oranı)
  aksi halde                                                   → belirsiz

Varsayılan eşikler simülasyon ölçümünden: 4 insan modeli × 3 yatış × 4 mesafe × 5 yön, 4 kamera
yüksekliği (2464 kare). Ayakta insanda gövde açısı ≤ 4°, kutu oranı p95 1.17; yatan insanda
gövde açısı medyan 83°, kutu oranı medyan 2.75. Bu eşiklerle ayakta insan (512 kare) ve kutularda
(192 kare) yanlış "yatan" sayısı 0.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

LYING, UPRIGHT, UNKNOWN = 'lying', 'upright', 'unknown'
L_SHOULDER, R_SHOULDER, L_HIP, R_HIP = 5, 6, 11, 12


@dataclass
class PostureParams:
    torso_lying_deg: float = 45.0
    torso_upright_deg: float = 30.0
    aspect_lying: float = 1.5
    kp_conf: float = 0.5


def torso_angle(keypoints: Optional[Sequence[Sequence[float]]],
                kp_conf: float) -> Optional[float]:
    """Gövde ekseninin düşeyle açısı (derece); anahtar noktalar güvenilmezse None."""
    if keypoints is None or len(keypoints) <= R_HIP:
        return None
    pts = [keypoints[i] for i in (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)]
    if min(p[2] for p in pts) < kp_conf:
        return None
    sx, sy = (pts[0][0] + pts[1][0]) / 2.0, (pts[0][1] + pts[1][1]) / 2.0
    hx, hy = (pts[2][0] + pts[3][0]) / 2.0, (pts[2][1] + pts[3][1]) / 2.0
    return math.degrees(math.atan2(abs(hx - sx), abs(hy - sy)))


def classify(width: float, height: float, keypoints: Optional[Sequence[Sequence[float]]],
             p: PostureParams) -> Tuple[str, Optional[float], float, str]:
    """Döndürür: (duruş, gövde açısı ya da None, kutu oranı, kararı veren ipucu)."""
    aspect = width / height if height > 0 else 0.0
    angle = torso_angle(keypoints, p.kp_conf)
    if angle is not None:
        if angle >= p.torso_lying_deg:
            return LYING, angle, aspect, 'torso'
        if angle <= p.torso_upright_deg and aspect < p.aspect_lying:
            return UPRIGHT, angle, aspect, 'torso'
    if aspect >= p.aspect_lying:
        return LYING, angle, aspect, 'aspect'
    return UNKNOWN, angle, aspect, 'none'
