#!/usr/bin/env python3
"""
factory.sdf üretici: worlds/factory.sdf bu betiğin çıktısıdır.

Engel yerleşimi (raf/kolon/palet/duvar) değiştirilecekse SDF'i elle değil buradan üretin;
map_quality_check.py ve scan_alignment_check.py referans geometriyi SDF kutularından okur.
Çıktı, Aşama 4 sonrası depodaki factory.sdf ile bayt bayt aynıdır.

Kullanım:
  ros2 run fsb_gazebo generate_factory_world.py <çıktı.sdf>
  # ör. src/fsb_gazebo/worlds/factory.sdf (değiştirmeden önce yedek alın), sonra paketi derleyin
"""

import argparse
from typing import List, Tuple

RGB = Tuple[float, float, float]


def box(name: str, x: float, y: float, z: float,
        sx: float, sy: float, sz: float, rgb: RGB) -> str:
    r, g, b = rgb
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
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


def world_sdf() -> str:
    return f"""<?xml version="1.0"?>
<!--
  factory.sdf — Fabrika temalı basit dünya (Gazebo Harmonic / gz-sim 8)
  Salon: 20 m x 14 m, duvar 3 m. Ana koridor: |y| < 2.6 m boyunca boş (robot başlangıcı 0,0).
  Raf sıraları y = ±3.0 ve ±5.5; ara koridorlar x ∈ [-4,-2] ve [2,4]; kolonlar; doğu ucunda paletler.
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
          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>
{''.join(models())}
  </world>
</sdf>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('output', help='yazılacak .sdf yolu')
    args = ap.parse_args()
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(world_sdf())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
