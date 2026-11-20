"""
patrol_planner çıktı dosyası: makineye/kullanıcıya özel mutlak yol içermemeli.

Waypoint yaml'ı repoda tutulur; içindeki harita ve robot_params yolları çıktı dosyasının dizinine
göre göreli yazılmalı ve o dizinden çözüldüğünde gerçek dosyalara işaret etmelidir.
"""

import os
import shutil

from fsb_patrol.coverage_model import load_map, load_robot_params, save_map
from fsb_patrol.map_variants import VARIANTS
from fsb_patrol.patrol_planner import plan, write_yaml
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(PKG, '..', 'fsb_navigation', 'maps', 'factory.yaml')
PARAMS = os.path.join(PKG, 'config', 'robot_params.yaml')


def test_waypoint_yaml_has_only_relative_source_paths(tmp_path):
    if not os.path.exists(MAP):
        pytest.skip('factory haritası bulunamadı')
    # Repodaki düzenin kopyası: <ws>/src/fsb_navigation/maps, <ws>/src/fsb_patrol/config
    nav_maps = tmp_path / 'src' / 'fsb_navigation' / 'maps'
    cfg = tmp_path / 'src' / 'fsb_patrol' / 'config'
    nav_maps.mkdir(parents=True)
    cfg.mkdir(parents=True)
    map_yaml = str(nav_maps / 'factory.yaml')
    save_map(VARIANTS['kirp_kd_ceyrek'](load_map(MAP)), map_yaml)   # küçük harita: hızlı
    params = str(cfg / 'robot_params.yaml')
    shutil.copy(PARAMS, params)
    out = str(cfg / 'waypoints.yaml')

    res = plan(load_map(map_yaml), load_robot_params(params))
    write_yaml(res, out, params)

    with open(out, encoding='utf-8') as f:
        text = f.read()
    doc = yaml.safe_load(text)
    assert doc['map']['yaml'] == os.path.join('..', '..', 'fsb_navigation', 'maps',
                                              'factory.yaml')
    assert doc['robot_params'] == 'robot_params.yaml'
    for rel in (doc['map']['yaml'], doc['robot_params']):
        assert not os.path.isabs(rel)
        assert os.path.isfile(os.path.join(os.path.dirname(out), rel))
    # Hiçbir mutlak yol ya da ev dizini metne sızmamalı
    assert str(tmp_path) not in text
    assert os.path.expanduser('~') not in text
    assert doc['waypoints'], 'planlayıcı waypoint üretmedi'
