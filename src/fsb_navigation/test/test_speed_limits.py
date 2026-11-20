"""
Nav2 hız/ivme sınırları ↔ diff_drive_controller sınırları tutarlılığı.

Nav2'nin ürettiği komut, controller'ın son güvenlik katmanı olan sınırları aşmamalıdır. Aşarsa
controller komutu sessizce kırpar ve Nav2'nin hız/zaman tahminleri gerçeği yansıtmaz (ör. RPP
0.22 m/s isterken controller 0.2 m/s'de sınırlıyordu).
"""

import os

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


def _load(pkg: str, rel: str) -> dict:
    with open(os.path.join(get_package_share_directory(pkg), rel), encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def ddc() -> dict:
    return _load('fsb_control', 'config/controllers.yaml')[
        'diff_drive_controller']['ros__parameters']


@pytest.fixture(scope='module')
def nav() -> dict:
    return _load('fsb_navigation', 'config/nav2_params.yaml')


def test_controller_limits_are_symmetric_and_enabled(ddc):
    assert ddc['linear.x.max_velocity'] > 0
    assert ddc['linear.x.min_velocity'] == pytest.approx(-ddc['linear.x.max_velocity'])
    assert ddc['angular.z.min_velocity'] == pytest.approx(-ddc['angular.z.max_velocity'])


def test_rpp_within_controller_limits(nav, ddc):
    fp = nav['controller_server']['ros__parameters']['FollowPath']
    assert fp['desired_linear_vel'] <= ddc['linear.x.max_velocity']
    assert fp['rotate_to_heading_angular_vel'] <= ddc['angular.z.max_velocity']
    assert fp['max_angular_accel'] <= ddc['angular.z.max_acceleration']
    assert fp['min_approach_linear_velocity'] <= fp['desired_linear_vel']


def test_smoother_within_controller_limits(nav, ddc):
    vs = nav['velocity_smoother']['ros__parameters']
    vmax, _, wmax = vs['max_velocity']
    vmin, _, wmin = vs['min_velocity']
    amax, _, alpha_max = vs['max_accel']
    dmin, _, dalpha_min = vs['max_decel']
    assert vmax <= ddc['linear.x.max_velocity']
    assert vmin >= ddc['linear.x.min_velocity']
    assert wmax <= ddc['angular.z.max_velocity']
    assert wmin >= ddc['angular.z.min_velocity']
    assert amax <= ddc['linear.x.max_acceleration']
    assert alpha_max <= ddc['angular.z.max_acceleration']
    assert -dmin <= ddc['linear.x.max_acceleration']
    assert -dalpha_min <= ddc['angular.z.max_acceleration']


def test_smoother_does_not_limit_rpp_below_its_target(nav):
    # Smoother RPP'nin hedef hızını kırpmamalı; aksi halde RPP ayarı etkisiz kalır.
    fp = nav['controller_server']['ros__parameters']['FollowPath']
    vs = nav['velocity_smoother']['ros__parameters']
    assert fp['desired_linear_vel'] <= vs['max_velocity'][0]


def test_smoother_output_is_not_stale(nav, ddc):
    # Smoother eski damgalı komut yinelerse controller onları cmd_vel_timeout sonrası reddeder;
    # girdi kesilince smoother en geç controller'ın zaman aşımında sıfıra inmeli.
    vs = nav['velocity_smoother']['ros__parameters']
    assert vs['stamp_smoothed_velocity_with_smoothing_time'] is True
    assert vs['velocity_timeout'] <= ddc['cmd_vel_timeout']


def test_behaviors_within_controller_limits(nav, ddc):
    b = nav['behavior_server']['ros__parameters']
    assert b['max_rotational_vel'] <= ddc['angular.z.max_velocity']
    assert b['min_rotational_vel'] <= b['max_rotational_vel']
