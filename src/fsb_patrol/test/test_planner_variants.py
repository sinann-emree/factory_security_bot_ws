"""
Genelleme regresyon testi: patrol_planner haritanın sentetik varyantlarında (Gazebo'suz).

Her varyantta: hata yok, kapsama ≥ target_coverage, waypoint'ler geçerli (traversable,
maliyet sınırı), hepsi birbirine ulaşılabilir, tespit edilen tüm koridor segmentlerine girilir.
"""

import os

from fsb_patrol.coverage_model import load_map, save_map
from fsb_patrol.map_variants import check_variant, VARIANTS
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(PKG, '..', 'fsb_navigation', 'maps', 'factory.yaml')
PARAMS = os.path.join(PKG, 'config', 'robot_params.yaml')


@pytest.mark.parametrize('name', list(VARIANTS))
def test_variant(name, tmp_path):
    if not os.path.exists(MAP):
        pytest.skip('factory haritası bulunamadı')
    gm = VARIANTS[name](load_map(MAP))
    ypath = str(tmp_path / f'{name}.yaml')
    save_map(gm, ypath)
    r = check_variant(name, load_map(ypath), PARAMS)
    assert r['finite'], 'ulaşılamayan waypoint çifti'
    assert r['valid'], 'geçersiz waypoint (traversable/maliyet)'
    assert r['coverage_total'] >= 0.95
    assert r['corridor_segments_entered'] == r['corridor_segments']
    assert r['ok']
