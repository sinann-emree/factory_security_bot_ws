"""
URDF ↔ controllers.yaml tutarlılığı ve fiziksel geçerlilik testleri.

Proje tasarım kurallarını build testinde zorlar: LiDAR 180° dönük montaj, fiziksel
parametrelerin URDF ↔ controller arasında birebir tutarlılığı ve geçerli atalet matrisleri.
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

TOL = 1e-9


def _render(use_sim: str) -> ET.Element:
    path = os.path.join(
        get_package_share_directory('fsb_description'), 'urdf', 'fsb_robot.urdf.xacro')
    doc = xacro.process_file(path, mappings={'use_sim': use_sim})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def urdf() -> ET.Element:
    return _render('true')


@pytest.fixture(scope='module')
def ddc() -> dict:
    path = os.path.join(
        get_package_share_directory('fsb_control'), 'config', 'controllers.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)['diff_drive_controller']['ros__parameters']


def _joint(urdf: ET.Element, name: str) -> ET.Element:
    j = urdf.find(f"joint[@name='{name}']")
    assert j is not None, name
    return j


def _xyz(elem: ET.Element) -> list:
    return [float(v) for v in elem.find('origin').get('xyz').split()]


def _wheel_radius(urdf: ET.Element) -> float:
    vis = urdf.find("link[@name='left_wheel_link']/visual/geometry/cylinder")
    col = urdf.find("link[@name='left_wheel_link']/collision/geometry/sphere")
    assert vis is not None and col is not None
    r = float(vis.get('radius'))
    assert abs(float(col.get('radius')) - r) < TOL, 'çarpışma küresi != görsel yarıçap'
    return r


def test_wheel_radius_matches_controller(urdf, ddc):
    assert abs(_wheel_radius(urdf) - ddc['wheel_radius']) < TOL


def test_wheel_separation_matches_controller(urdf, ddc):
    sep = _xyz(_joint(urdf, 'left_wheel_joint'))[1] - _xyz(_joint(urdf, 'right_wheel_joint'))[1]
    assert abs(sep - ddc['wheel_separation']) < TOL


def test_controller_frames_and_flags(ddc):
    assert ddc['base_frame_id'] == 'base_footprint'
    assert ddc['enable_odom_tf'] is False
    assert 'use_stamped_vel' not in ddc


def test_wheels_touch_ground(urdf):
    base_z = _xyz(_joint(urdf, 'base_joint'))[2]
    r = _wheel_radius(urdf)
    for side in ('left', 'right'):
        bottom = base_z + _xyz(_joint(urdf, f'{side}_wheel_joint'))[2] - r
        assert abs(bottom) < TOL, f'{side} tekerlek alt noktası z={bottom}'


def test_caster_touches_ground(urdf):
    base_z = _xyz(_joint(urdf, 'base_joint'))[2]
    r = float(urdf.find("link[@name='caster_link']/collision/geometry/sphere").get('radius'))
    bottom = base_z + _xyz(_joint(urdf, 'caster_joint'))[2] - r
    assert abs(bottom) < TOL, f'caster alt noktası z={bottom}'


def test_wheel_axes(urdf):
    for side in ('left', 'right'):
        j = _joint(urdf, f'{side}_wheel_joint')
        assert j.get('type') == 'continuous'
        assert j.find('axis').get('xyz').split() == ['0', '1', '0']


def test_lidar_rotated_180(urdf):
    rpy = [float(v) for v in _joint(urdf, 'lidar_joint').find('origin').get('rpy').split()]
    assert rpy[0] == 0.0 and rpy[1] == 0.0
    assert abs(rpy[2] - math.pi) < 1e-6


def test_imu_on_rotation_axis(urdf):
    x, y, _ = _xyz(_joint(urdf, 'imu_joint'))
    assert abs(x) < TOL and abs(y) < TOL


def test_tf_tree(urdf):
    expected = {
        'base_joint': ('base_footprint', 'base_link'),
        'left_wheel_joint': ('base_link', 'left_wheel_link'),
        'right_wheel_joint': ('base_link', 'right_wheel_link'),
        'caster_joint': ('base_link', 'caster_link'),
        'lidar_joint': ('base_link', 'lidar_link'),
        'camera_joint': ('base_link', 'camera_link'),
        'imu_joint': ('base_link', 'imu_link'),
        'camera_optical_joint': ('camera_link', 'camera_optical_link'),
    }
    actual = {
        j.get('name'): (j.find('parent').get('link'), j.find('child').get('link'))
        for j in urdf.findall('joint')
    }
    assert actual == expected


# Saf koordinat çerçeveleri (kütlesiz): kök link ve kamera optik çerçevesi
FRAME_ONLY_LINKS = {'base_footprint', 'camera_optical_link'}


def test_inertials_valid(urdf):
    for link in urdf.findall('link'):
        name = link.get('name')
        if name in FRAME_ONLY_LINKS:
            assert link.find('inertial') is None, f'{name}: saf çerçeve, inertial olmamalı'
            continue
        inertial = link.find('inertial')
        assert inertial is not None, f'{name}: inertial yok'
        assert float(inertial.find('mass').get('value')) > 0.0, name
        i = inertial.find('inertia')
        ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
        for k in ('ixy', 'ixz', 'iyz'):
            assert float(i.get(k)) == 0.0, f'{name}: {k} != 0'
        assert min(ixx, iyy, izz) > 0.0, f'{name}: pozitif tanımlı değil'
        assert ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx, \
            f'{name}: üçgen eşitsizliği'


def test_sensor_frame_ids_match_links(urdf):
    links = {lk.get('name') for lk in urdf.findall('link')}
    sensors = {s.get('name'): s for s in urdf.iter('sensor')}
    assert set(sensors) == {'lidar', 'camera', 'imu'}
    expected = {'lidar': 'lidar_link', 'camera': 'camera_optical_link', 'imu': 'imu_link'}
    for name, frame in expected.items():
        assert sensors[name].find('gz_frame_id').text == frame
        assert frame in links


def test_camera_lens_in_front_of_chassis(urdf):
    cam_x = _xyz(_joint(urdf, 'camera_joint'))[0]
    lens_x = cam_x + _xyz(_joint(urdf, 'camera_optical_joint'))[0]
    col = urdf.find("link[@name='base_link']/collision")
    front = (float(col.find('origin').get('xyz').split()[0])
             + float(col.find('geometry/box').get('size').split()[0]) / 2.0)
    assert lens_x > front, f'lens x={lens_x} gövde ön yüzü={front}'


def test_no_sensors_without_sim():
    assert not list(_render('false').iter('sensor'))


def test_ros2_control_plugin_selection():
    def plugin(use_sim):
        return _render(use_sim).find('ros2_control/hardware/plugin').text.strip()
    assert plugin('true') == 'gz_ros2_control/GazeboSimSystem'
    assert plugin('false') != 'gz_ros2_control/GazeboSimSystem'
