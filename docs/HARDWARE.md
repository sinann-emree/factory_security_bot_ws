# Hardware design notes

Status: **no physical robot has been built yet.** This document records the intended parts, the
technical requirements derived from the simulation model, and the calculations behind the motor
choice. Values marked **unverified** have not been confirmed by measurement or by a manufacturer
datasheet. Values marked **[assumption]** are modelling assumptions.

## 1. Relation to Andino

The robot follows the dimensions of [Andino](https://github.com/Ekumen-OS/andino) (Ekumen-OS,
BSD-3-Clause, reference commit `974264b`), a small open differential-drive platform:

| Item | Source | Use in this project |
|---|---|---|
| Wheel radius 0.0331 m, wheel separation 0.128 m, chassis, caster, sensor offsets | `andino_description/config/andino/*.yaml` | Re-entered in `src/fsb_description/urdf/fsb_properties.xacro` with a source comment per value |
| 3D-printed chassis (STL) | `andino_hardware/printing_model/` | Printed as-is for Plan A motors; **not** stored in this repository (see `ATTRIBUTIONS.md`) |
| Arduino Nano firmware (motor PWM, quadrature encoders, PID, serial shell) | `andino_firmware/` | Built from the pinned commit as an external dependency; changes only as patch files (`docs/firmware.md`) |
| Encoder resolution 585 ticks/wheel revolution (DG01D-E) | `andino_description/config/andino/hardware.yaml:6` | Plan A value for `enc_ticks_per_rev` |

## 2. Intended parts

| Function | Part | Key specification | Notes |
|---|---|---|---|
| Compute | Raspberry Pi 4 B (4 GB) | Ubuntu 24.04 arm64, ROS 2 Jazzy (REP 2000 Tier 1) | YOLO is expected to run off-board (see `docs/PERCEPTION.md`) |
| Motor controller MCU | Arduino Nano (ATmega328P) | 57600 baud ASCII protocol, PCINT quadrature decoding | Same board package as Andino |
| Motor driver | L298N dual H-bridge | 2 A per channel; saturation drop 1.8 V typ / 3.2 V max at 1 A (ST L298 datasheet) | Bipolar bridge: large voltage drop |
| Motors | **Plan A:** DG01D-E (TT format, encoder) · **Plan B:** N20 12 V 300 RPM with encoder | see §4 | |
| LiDAR | Slamtec RPLIDAR C1 | 360°, 10 Hz typ., 0.05–12 m, 460800 baud | Jazzy driver must be built from source (C1 support not in released `rplidar_ros`); mounted rotated 180° like the simulated sensor |
| Camera | USB UVC webcam (e.g. Logitech C270) | 55° diagonal FoV (≈ 45–49° horizontal, crop mode **unverified**) | Simulation uses 62.2° HFOV; `camera_hfov` must be updated to the measured value |
| IMU | MPU6050 (GY-521) on the Pi I²C bus | 6-axis | No released Jazzy driver; a small `fsb_*` node is planned. Andino firmware only supports BNO055 |
| Battery | 3S Li-ion (3 × 18650) + 3S BMS | 11.1 V nominal, 12.6 V full | Estimated runtime ~2 h **[assumption: 10–12 W average]** |
| 5 V supply | 5 V / 3 A buck converter | Pi 4 needs 3 A continuous | Sustained 3 A from LM2596-class modules **unverified** |

## 3. Speed requirement

Maximum linear speed is **0.2 m/s** (`controllers.yaml` `linear.x.max_velocity`, Nav2 RPP
`desired_linear_vel` and velocity smoother). Angular limits: 0.8 rad/s in Nav2, 1.0 rad/s in the
controller. The lower speed was chosen so that both motor plans meet the requirement with margin;
in simulation the same patrol leg took 14 % longer than at 0.22 m/s (22.3 s → 25.5 s).

Wheel speed `n = v / (2π r) · 60`, r = 0.0331 m. Outer wheel while turning: `v + ω · b / 2`,
b = 0.128 m.

| Case | Wheel speed | Required RPM | ÷0.85 margin | ÷0.80 margin |
|---|---|---|---|---|
| Straight, 0.2 m/s | 0.200 m/s | 57.7 | 67.9 | **72.1** |
| 0.2 m/s + 0.8 rad/s (Nav2 limit) | 0.251 m/s | 72.5 | 85.3 | **90.6** |
| 0.2 m/s + 1.0 rad/s (controller limit) | 0.264 m/s | 76.2 | 89.6 | 95.2 |

**Requirement:** no-load speed at the actual motor voltage ≥ ~72 RPM straight and ≥ ~91 RPM while
turning (20 % load margin).

Motor voltage after the L298N **[assumption: no-load speed ∝ voltage]**:

| Supply / drop | Motor voltage |
|---|---|
| 3S full 12.6 V, typical drop | 10.8 V |
| 3S nominal 11.1 V, typical drop | 9.3 V |
| 3S nominal, maximum drop | 7.9 V |
| 3S at 3.3 V/cell (9.9 V), maximum drop | 6.7 V |

## 4. Motor choice

### Plan A — DG01D-E (primary)
- TT-format gear motor with Hall encoder, 1:48, 90 RPM at 4.5 V, rated 3–9 V (manufacturer
  datasheet DS-16413).
- Fits the Andino motor bay, wheel hub (5.5 × 3.6 mm double-D) and firmware without changes.
  Measured from the Andino STL: 2 × Ø3 mm holes 17.5 mm apart, 19.7 mm body slot, shaft axis
  20.6 mm from the hole line, shaft height 11.25 mm.
- Speed: linear estimate ≈ 158 RPM at 7.9 V → meets both requirements.
- Required change: the 3S pack can put more than 9 V on the motor, so the firmware PWM ceiling
  (`Motor::kMaxSpeed`) must be lowered with a patch (`docs/firmware.md` §5).
- Encoder: 585 ticks per wheel revolution (Andino value). At 0.2 m/s: 563 ticks/s per wheel,
  ≈ 19 ticks per 30 Hz PID cycle — the range the firmware PID was tuned for.

### Plan B — N20 12 V 300 RPM with encoder (fallback)
- 12 V rating → no PWM ceiling needed. ≈ 233 RPM at 9.3 V and ≈ 168 RPM at 6.7 V → speed
  requirement met with a large margin (0.2 m/s uses ~25 % of the no-load speed).
- **Requires new printed parts:** an N20 clamp bracket screwed into the existing M3 holes with the
  shaft at z = 11.25 mm, and a hub for the 3 mm D shaft. Not designed yet; when added they will be
  separate files with a note that they are derived from the Andino chassis.
- **Unverified:** encoder PPR, gear ratio, shaft diameter/shape, torque, stall current.
- The limiting factor is **torque**, not speed (§5).
- Firmware PID gains must be retuned because the encoder resolution differs. Example
  **[assumption: 7 PPR × 1:100 × 4 = 2800 ticks/rev]**: 2.7 k ticks/s per wheel at 0.2 m/s,
  ≈ 5.4 k interrupts/s for both wheels; whether the Nano's PCINT ISR keeps up is **unverified**.

### Rejected alternatives
| Class | Reason |
|---|---|
| 25 mm 12 V gear motors with encoder (e.g. JGA25-370 class, 100–170 RPM) | Encoder version not available locally; Ø25 body does not fit the 19.7 mm slot; shaft axis must rise above 11.25 mm; 4 mm D shaft needs a new hub |
| 37 mm 12 V gear motors (e.g. JGB37-520, 110 RPM) | Body too large for the 48 mm tall chassis, motor bay must be redesigned; ≈ 85 RPM at 9.3 V is below the 91 RPM turning requirement |
| Larger 25–37 mm motors with high stall current (3–7 A) | Exceed the L298N 2 A/channel limit |

## 5. Torque estimate

**[assumption]** mass 3.37 kg (sum of link masses in the simulation URDF), rolling resistance
coefficient 0.03, acceleration 0.5 m/s² (controller limit), both wheels share the load equally.

| Case | Total force | Per wheel | Torque per wheel (r = 0.0331 m) |
|---|---|---|---|
| Flat floor, accelerating at 0.5 m/s² | m(a + g·Crr) = 2.68 N | 1.34 N | 0.044 N·m ≈ **0.45 kg·cm** |
| Same + 5° ramp | + m·g·sin 5° = 5.56 N | 2.78 N | 0.092 N·m ≈ **0.94 kg·cm** |

With a safety factor of 2 the motor should deliver about **0.9–1.9 kg·cm** per wheel at the
operating speed. Plan A: DG01D-E torque at the capped voltage is **unverified**. Plan B: the only
seller figure available is "0–1.5 kg·cm" for the 100 RPM variant; the 300 RPM variant has a lower
gear ratio and therefore less torque, so Plan B is **marginal** and must be bench-tested before use.

## 6. Electrical constraints
- **Motor driver current:** TT and N20 class motors stay below the L298N 2 A/channel limit.
- **Encoder interface:** Andino firmware uses pin-change interrupts with ×4 quadrature decoding;
  A and B of one encoder must be on the same port (left: PORTD, right: PORTC).
- **USB budget:** RPLIDAR C1 230 mA + camera + Nano must stay below the Pi 4's 1.2 A total;
  camera and Nano current **unverified**.
- **Nano power:** from the Pi over USB, not from the L298N on-board 5 V regulator.

## 7. Software changes needed for real hardware (stage 7, planned)
1. `fsb_hardware`: ros2_control `SystemInterface` speaking the Andino serial protocol
   (`setspd` ticks/s, `getenc`), selected by `use_sim:=false` in the xacro.
2. `enc_ticks_per_rev` from measurement (`getenc` while turning the wheel 10 revolutions).
3. `wheel_radius` / `wheel_separation` re-measured; `test_params_consistency.py` keeps URDF and
   controller in sync.
4. LiDAR driver with `frame_id:=lidar_link`; verify the 180° mounting with `scan_alignment_check.py`.
5. MPU6050 node with gyro bias calibration at start-up (bias directly becomes yaw drift in the EKF).
6. `camera_hfov` from measurement; camera height ≥ 30–60 cm recommended for lying-person detection
   (`docs/PERCEPTION.md`).

## 8. Unverified items
- Plan B PPR, gear ratio, shaft, torque, stall current; DG01D-E torque at the capped voltage.
- Locating pin seat (Ø3.8 mm) in the printed motor bay.
- Nano interrupt load with high-resolution encoders.
- RPLIDAR C1 source build on Jazzy/arm64; webcam FoV in 640×480 mode.
- 5 V buck sustained current, USB current budget, battery runtime.
