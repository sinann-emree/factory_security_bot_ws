# Attributions and third-party components

The source code in this repository is licensed under Apache-2.0 (`LICENSE`). The components below
are **not copied** into this repository; they are installed via apt/pip/Gazebo Fuel or used from a
separate checkout. Licence information was read on 2026-11-20 from each package's `package.xml`,
package metadata or the Fuel API.

## Andino (Ekumen-OS/andino)

| Field | Value |
|---|---|
| Repository | https://github.com/Ekumen-OS/andino |
| Reference commit | `974264bdefd6ab939ba487ffa95ce3c1525285ff` |
| Licence | BSD 3-Clause, "Copyright (c) 2023, Ekumen Inc." (per package: `andino_description/LICENSE`, `andino_firmware/LICENSE`, `andino_hardware/LICENSE`, …; no LICENSE at the repository root) |

What this project uses from Andino:

1. **Robot dimensions and physical parameters** (wheel radius and separation, chassis, caster,
   sensor positions, masses). The values were re-entered from scratch in
   `src/fsb_description/urdf/fsb_properties.xacro` and related yaml files; each value carries an
   `andino <file>:<line>` source comment. No Andino file was copied.
2. **Firmware (Arduino Nano):** `andino_firmware` is built from the pinned commit as an **external
   dependency**; its source is not in this repository. Any change is kept only as a patch file
   (`src/fsb_hardware/firmware/patches/`, none yet). Patch files are derivative works and keep the
   original BSD-3 notice. Details: `docs/firmware.md`.
3. **Serial protocol** (57600 baud; `getenc`, `setspd`, `setpid`, … commands): documented with
   file:line references in `docs/firmware.md` and summarised in `docs/HARDWARE.md`.
4. **Chassis STL files (3D printing):** `andino_hardware/printing_model/`
   (`chassis/base_chassis.stl`, `lidar_chassis.stl`, `battery_chassis.stl`, `arduino_lock_base.stl`,
   `raspi_cam_mount/*.stl`). Licence: `andino_hardware/LICENSE` (BSD 3-Clause, Ekumen Inc.) and
   `andino_hardware/package.xml` (`BSD Clause 3`). No separate licence statement specific to the
   STL files was found; they are assumed to be covered by the package licence. **The STL files are
   not included in this repository**; download them from
   https://github.com/Ekumen-OS/andino/tree/974264bdefd6ab939ba487ffa95ce3c1525285ff/andino_hardware/printing_model
   Modified mounts (e.g. the Plan B N20 bracket, `docs/HARDWARE.md` §4) will be added as separate
   files with a note that they are derived from the Andino chassis; none exist yet.

### Firmware parts derived from ros_arduino_bridge

As stated in their headers, several Andino firmware files are inspired by
[hbrobotics/ros_arduino_bridge](https://github.com/hbrobotics/ros_arduino_bridge) and carry its
licence: BSD, "Copyright (c) 2012, Patrick Goebel". Files with this header at Andino commit
`974264b`:

- `andino_firmware/src/app/app.cpp`, `src/app/pid.cpp`
- `andino_firmware/src/drivers/encoder.cpp`, `src/drivers/motor.cpp`
- `andino_firmware/include/andino/app/commands.h`, `include/andino/drivers/encoder.h`,
  `include/andino/drivers/motor.h`

These files are not in this repository. If the firmware is distributed as a binary
(`firmware.hex`), both BSD notices (Ekumen Inc. and Patrick Goebel) and their licence texts must
accompany it.

## ROS 2 and simulation (apt, ROS 2 Jazzy)

| Component | Used for | Licence (`package.xml`) | Version tested |
|---|---|---|---|
| ROS 2 Jazzy (rclpy, tf2, launch, …) | Framework | Apache-2.0 (mostly) | — |
| Nav2 (`nav2_*`) | Navigation: Smac planner, Regulated Pure Pursuit, velocity smoother, collision monitor | Apache-2.0; `nav2_amcl` LGPL-2.1-or-later | 1.3.13 |
| SLAM Toolbox | Mapping | LGPL | 2.8.5 |
| robot_localization | EKF (wheel odometry + IMU) | Apache-2.0 | 3.8.3 |
| ros2_control (`controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`) | Drive control | Apache-2.0 | 4.x |
| gz_ros2_control | Simulation hardware interface | Apache-2.0 | 1.2.20 |
| ros_gz (`ros_gz_sim`, `ros_gz_bridge`) | Gazebo ↔ ROS bridge | Apache-2.0 | 1.0.24 |
| Gazebo Harmonic (gz-sim 8) | Simulator | Apache-2.0 | 8.15.0 |
| robot_state_publisher, xacro | Robot model | BSD | — |
| cv_bridge, image_transport | Images | Apache-2.0 / BSD | — |

The LGPL components (AMCL, SLAM Toolbox) run as separate processes; this repository neither
contains nor links their code.

## Ultralytics YOLO (AGPL-3.0)

| Field | Value |
|---|---|
| Package | `ultralytics` (pip, separate venv created by `scripts/setup_perception_venv.sh`) |
| Licence | AGPL-3.0 (alternative: Ultralytics Enterprise licence) |
| Version tested | 8.4.201 |
| Model weights | `yolov8n-pose.pt` (Ultralytics, same terms); **not** in this repository, downloaded by the setup script |

`fsb_perception` does not contain Ultralytics code; it only imports it at runtime, so the source in
this repository stays Apache-2.0. A system that is **distributed or offered as a network service**
together with `fsb_perception` is subject to AGPL-3.0 obligations (e.g. providing source code) or
needs Ultralytics' commercial licence.

## Gazebo Fuel models

The models are not stored in this repository; the world files reference them with
`<include><uri>` and Gazebo downloads them to `~/.gz/fuel` on first start. Licences were read from
the Fuel API (`https://fuel.gazebosim.org/1.0/<owner>/models/<name>`, field `license_name`).

| Model | Owner | Used in | Licence |
|---|---|---|---|
| Walking person | OpenRobotics | `factory_person.sdf`, `factory_negative.sdf` | CC0-1.0 |
| Casual female | OpenRobotics | `factory_lying.sdf` | CC0-1.0 |
| Standing person | OpenRobotics | option of `generate_factory_world.py` only | CC0-1.0 |
| Rescue Randy | OpenRobotics | option of `generate_factory_world.py` only | CC-BY-4.0 (attribution required if used) |

## Python dependencies (apt)

numpy, scipy, matplotlib, Pillow, PyYAML, psutil, OpenCV (`python3-opencv`): Ubuntu 24.04
packages under their own permissive licences (BSD / MIT / PSF / Apache-2.0 style).
