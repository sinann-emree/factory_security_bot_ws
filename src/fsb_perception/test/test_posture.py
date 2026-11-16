"""Duruş sınıflandırma birim testleri (ROS / torch gerekmez)."""

from fsb_perception.posture import classify, LYING, PostureParams, UNKNOWN, UPRIGHT

P = PostureParams()


def kp(shoulder, hip, conf=0.9):
    """17 anahtar nokta; yalnızca omuzlar (5, 6) ve kalçalar (11, 12) anlamlı."""
    pts = [[0.0, 0.0, 0.0] for _ in range(17)]
    (sx, sy), (hx, hy) = shoulder, hip
    pts[5], pts[6] = [sx - 5, sy, conf], [sx + 5, sy, conf]
    pts[11], pts[12] = [hx - 5, hy, conf], [hx + 5, hy, conf]
    return pts


def test_upright_person():
    posture, angle, aspect, cue = classify(60, 200, kp((100, 50), (100, 130)), P)
    assert posture == UPRIGHT and cue == 'torso' and angle < 5 and aspect < 0.5


def test_lying_by_torso_axis():
    posture, angle, _, cue = classify(200, 60, kp((50, 100), (130, 100)), P)
    assert posture == LYING and cue == 'torso' and angle > 85


def test_lying_by_aspect_when_keypoints_weak():
    posture, angle, _, cue = classify(200, 60, kp((50, 100), (130, 100), conf=0.2), P)
    assert posture == LYING and cue == 'aspect' and angle is None


def test_lying_by_aspect_without_keypoints():
    assert classify(300, 100, None, P)[0] == LYING


def test_foreshortened_wide_box_is_lying():
    # gövde ekseni görüş doğrultusunda (açı küçük) ama kutu belirgin biçimde yatık
    posture, _, _, cue = classify(180, 90, kp((100, 50), (100, 70)), P)
    assert posture == LYING and cue == 'aspect'


def test_ambiguous_is_unknown():
    # anahtar nokta yok, kutu ne dik ne yatık: alarm üretmemeli
    assert classify(100, 100, None, P)[0] == UNKNOWN
    # gövde 35° eğik (eşikler arasında), kutu dar
    assert classify(80, 120, kp((100, 50), (135, 100)), P)[0] == UNKNOWN


def test_thresholds_are_parameters():
    strict = PostureParams(torso_lying_deg=80.0, aspect_lying=3.0)
    assert classify(100, 100, kp((50, 50), (100, 100)), P)[0] == LYING       # 45°
    assert classify(100, 100, kp((50, 50), (100, 100)), strict)[0] != LYING
