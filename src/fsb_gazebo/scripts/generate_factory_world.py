#!/usr/bin/env python3
"""
factory.sdf üretici: worlds/factory.sdf bu betiğin çıktısıdır.

Engel yerleşimi (raf/kolon/palet/duvar) değiştirilecekse SDF'i elle değil buradan üretin;
map_quality_check.py ve scan_alignment_check.py referans geometriyi SDF kutularından okur.
Çıktı, Aşama 4 sonrası depodaki factory.sdf ile bayt bayt aynıdır.

İnsan modelleri Gazebo Fuel'dan <include> edilir (statik). Model dosyaları depoda tutulmaz;
Gazebo ilk açılışta indirip ~/.gz/fuel altında önbelleğe alır (ilk açılışta internet gerekir).
  --lying   YERDE YATAN insan (alarm hedefi). Fuel'da yerde yatan hazır model yok (hasta modelleri
            yatak üzerinde); ayakta modeller yatık pozla yerleştirilir (sırtüstü/yüzüstü/yan).
  --person  AYAKTA insan — yalnızca negatif test dünyası için (alarm üretmemeli).
  --box     0.5 m küp — negatif test (alarm üretmemeli).
Hiçbiri verilmezse çıktı factory.sdf ile bayt bayt aynıdır.

Kullanım:
  ros2 run fsb_gazebo generate_factory_world.py <çıktı.sdf>
  # ör. src/fsb_gazebo/worlds/factory.sdf (değiştirmeden önce yedek alın), sonra paketi derleyin
  ros2 run fsb_gazebo generate_factory_world.py <çıktı.sdf> --lying casual_female \
      --lying-pose -5.0 0.8 0.4 --lying-type supine   # worlds/factory_lying.sdf
  ros2 run fsb_gazebo generate_factory_world.py <çıktı.sdf> --person walking \
      --person-pose 9.2 0.0 3.14159265 --box 4.0 1.6  # worlds/factory_negative.sdf
"""

import argparse
import math
from typing import Dict, List, Tuple

RGB = Tuple[float, float, float]


def box(name: str, x: float, y: float, z: float,
        sx: float, sy: float, sz: float, rgb: RGB) -> str:
    r, g, b = rgb
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box>\
</geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>\
<specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>"""


HX, HY, WT, WH = 10.0, 7.0, 0.2, 3.0   # salon yarı uzunluk x/y, duvar kalınlığı, yüksekliği
WALL: RGB = (0.75, 0.75, 0.72)
SHELF: RGB = (0.15, 0.35, 0.65)
PILLAR: RGB = (0.55, 0.55, 0.55)
PALLET: RGB = (0.65, 0.45, 0.2)


def models() -> List[str]:
    m: List[str] = []
    # Dış duvarlar
    m.append(box('wall_north', 0, HY + WT / 2, WH / 2, 2 * HX + 2 * WT, WT, WH, WALL))
    m.append(box('wall_south', 0, -HY - WT / 2, WH / 2, 2 * HX + 2 * WT, WT, WH, WALL))
    m.append(box('wall_east', HX + WT / 2, 0, WH / 2, WT, 2 * HY, WH, WALL))
    m.append(box('wall_west', -HX - WT / 2, 0, WH / 2, WT, 2 * HY, WH, WALL))
    # Raf sıraları: 4 m uzun, 0.8 m derin, 2 m yüksek. Ana koridor |y| < 2.6 boş.
    for i, x in enumerate((-6.0, 0.0, 6.0)):
        for j, y in enumerate((3.0, -3.0, 5.5, -5.5)):
            m.append(box(f'shelf_{i}_{j}', x, y, 1.0, 4.0, 0.8, 2.0, SHELF))
    # Kolonlar (ara koridorlarda, SLAM için ayırt edici özellik)
    for k, (x, y) in enumerate(((-3.0, 4.25), (3.0, 4.25), (-3.0, -4.25), (3.0, -4.25))):
        m.append(box(f'pillar_{k}', x, y, WH / 2, 0.4, 0.4, WH, PILLAR))
    # Yükleme alanı palet istifleri (doğu ucu)
    m.append(box('pallet_0', 8.8, -1.6, 0.5, 1.0, 1.2, 1.0, PALLET))
    m.append(box('pallet_1', 8.8, 1.6, 0.35, 1.0, 1.2, 0.7, PALLET))
    return m


# Gazebo Fuel insan modelleri (OpenRobotics). yaw_offset: modelin yüzü dünya +x'e baksın diye
# eklenen sabit dönüş (modelin kendi ön yönü; kamera görüntüsünden doğrulandı).
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
# Dördü de kendi çerçevesinde -y'ye bakar → +90°. ("Male visitor" bir <actor>: statik
# yerleştirilemiyor, betiği pozu eziyor ve zemine gömülüyor; listeye alınmadı.)
PERSONS: Dict[str, Tuple[str, float]] = {
    'walking': (FUEL + 'Walking person', math.pi / 2),
    'standing': (FUEL + 'Standing person', math.pi / 2),
    'casual_female': (FUEL + 'Casual female', math.pi / 2),
    'rescue_randy': (FUEL + 'Rescue Randy', math.pi / 2),
}
PERSON_NAME = 'intruder_person'


def person(kind: str, x: float, y: float, yaw: float) -> str:
    """Fuel insan modeli; yaw = yüzün baktığı yön (dünya çerçevesi, rad)."""
    uri, yaw_offset = PERSONS[kind]
    return f"""
    <include>
      <name>{PERSON_NAME}</name>
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw + yaw_offset:.6f}</pose>
      <uri>{uri}</uri>
    </include>"""


# Yatış türü → (roll, pitch, baş yönüne eklenen yaw, zemin üstü kaldırma m). Modeller ayakta
# +z boyunca durur ve -y'ye bakar; köken ayak tabanındadır.
LIE: Dict[str, Tuple[float, float, float, float]] = {
    'supine': (-math.pi / 2, 0.0, -math.pi / 2, 0.13),   # sırtüstü
    'prone': (math.pi / 2, 0.0, math.pi / 2, 0.16),      # yüzüstü
    'side': (0.0, math.pi / 2, 0.0, 0.24),               # yan
}
LYING_NAME = 'lying_person'
BODY_LENGTH = 1.7


def lying_person(kind: str, cx: float, cy: float, heading: float, lie: str,
                 name: str = LYING_NAME) -> str:
    """Yerde yatan insan; (cx, cy) gövde ortası, heading = başın baktığı yön (rad)."""
    uri, _ = PERSONS[kind]
    roll, pitch, dyaw, z = LIE[lie]
    x = cx - 0.5 * BODY_LENGTH * math.cos(heading)   # ayak tabanı (model kökeni)
    y = cy - 0.5 * BODY_LENGTH * math.sin(heading)
    return f"""
    <include>
      <name>{name}</name>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z} {roll:.6f} {pitch:.6f} {heading + dyaw:.6f}</pose>
      <uri>{uri}</uri>
    </include>"""


def world_sdf(extra: str = '') -> str:
    return f"""<?xml version="1.0"?>
<!--
  factory.sdf — Fabrika temalı basit dünya (Gazebo Harmonic / gz-sim 8)
  Salon: 20 m x 14 m, duvar 3 m. Ana koridor: |y| < 2.6 m boyunca boş \
(robot başlangıcı 0,0).
  Raf sıraları y = ±3.0 ve ±5.5; ara koridorlar x ∈ [-4,-2] ve [2,4]; kolonlar; \
doğu ucunda paletler.
  Üretici: fsb_gazebo/scripts/generate_factory_world.py; elle düzenlenebilir.
  Sistem eklentileri: Physics, UserCommands (ros_gz_sim create için gerekli), SceneBroadcaster,
  Sensors (gpu_lidar + kamera, ogre2), Imu (Aşama 4).
-->
<sdf version="1.9">
  <world name="factory">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>
          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse>\
<specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>
{''.join(models())}{extra}
  </world>
</sdf>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('output', help='yazılacak .sdf yolu')
    ap.add_argument('--person', choices=sorted(PERSONS), help='eklenecek Fuel insan modeli')
    ap.add_argument('--person-pose', nargs=3, type=float, default=[9.2, 0.0, 3.14159265],
                    metavar=('X', 'Y', 'YAW'), help='konum (m) ve yüzün baktığı yön (rad)')
    ap.add_argument('--lying', choices=sorted(PERSONS), help='yerde yatan Fuel insan modeli')
    ap.add_argument('--lying-pose', nargs=3, type=float, default=[-5.0, 0.8, 0.4],
                    metavar=('X', 'Y', 'HEADING'), help='gövde ortası (m) ve baş yönü (rad)')
    ap.add_argument('--lying-type', choices=sorted(LIE), default='supine')
    ap.add_argument('--lying-at', nargs=5, action='append', default=[],
                    metavar=('MODEL', 'TYPE', 'X', 'Y', 'HEADING'),
                    help='ek yatan insan (yinelenebilir); adları lying_person_2, _3, ...')
    ap.add_argument('--box', nargs=2, type=float, metavar=('X', 'Y'),
                    help='0.5 m küp (negatif test)')
    args = ap.parse_args()
    extra = person(args.person, *args.person_pose) if args.person else ''
    if args.lying:
        extra += lying_person(args.lying, *args.lying_pose, args.lying_type)
    for k, (kind, lie, x, y, heading) in enumerate(args.lying_at, start=2):
        if kind not in PERSONS or lie not in LIE:
            ap.error(f'--lying-at: bilinmeyen model/tip: {kind} {lie}')
        extra += lying_person(kind, float(x), float(y), float(heading), lie,
                              f'{LYING_NAME}_{k}')
    if args.box:
        extra += box('neg_box', args.box[0], args.box[1], 0.25, 0.5, 0.5, 0.5, PALLET)
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(world_sdf(extra))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
