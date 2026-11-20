# Testing

Four levels. Levels 1–2 need no simulator and run in `colcon test` (and in CI). Levels 3–4 need the
perception venv and/or Gazebo and are run manually. All numbers are simulation measurements
(Gazebo Harmonic, 20 × 14 m factory world, WSL2, 16-core CPU, NVIDIA GPU). No real-robot
measurements exist yet.

Environment for every terminal:
```bash
cd ~/factory_security_bot_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=85        # same in every terminal
```

> **Speed note:** the maximum linear speed was lowered from 0.22 m/s (RPP) to **0.2 m/s** on
> 2026-11-20 (`docs/HARDWARE.md` §3). Level-4 rows without a date were measured at 0.22 m/s;
> rows re-measured at 0.2 m/s are marked.

## Level 1 — Lint and unit tests (`colcon test`)

```bash
colcon build
colcon test
colcon test-result --all --verbose
```

**Expected:** `0 errors, 0 failures` in every package. The only skipped test is `ament_copyright`
(template from `ros2 pkg create`; source files carry no copyright header).

No test reads or writes `~/.ros/fsb_security` (persistent incident list); tests that persist
state use pytest `tmp_path`. CI therefore starts from a clean state on every run.

| Package | Tests | What they check |
|---|---|---|
| all | flake8, pep257, xmllint, lint_cmake | Style, docstrings, XML and CMake format |
| `fsb_description` | `test_params_consistency.py` (14) | Wheel radius/separation identical in URDF and `controllers.yaml`; controller frames and flags; wheels and caster touch the ground; wheel axes; LiDAR joint rotated 180°; IMU on the rotation axis; TF tree; valid inertials (positive definite, triangle inequality); sensor `frame_id` = URDF link; ros2_control plugin selected by `use_sim` |
| `fsb_perception` | `test_posture.py` (7) | Posture rules: upright, lying by torso axis, lying by box aspect when keypoints are weak or missing, ambiguous → unknown, thresholds are parameters |
| `fsb_control` | `test_cmd_vel_watchdog.py` (3) | Real rclpy nodes in an isolated namespace: no output before the first command; commands forwarded unchanged; zero velocity `timeout` after commands stop (delay checked), repeated at `zero_rate`, `~/timed_out` = true; relay re-opens when commands return |
| `fsb_navigation` | `test_speed_limits.py` (6) | RPP, velocity smoother and behavior server speed/acceleration limits ≤ `diff_drive_controller` limits; smoother does not clip the RPP target; smoother output is re-stamped and times out no later than the controller (see "Known issues") |
| `fsb_patrol` | `test_planner_output.py` (1) | Waypoint yaml written by the planner references the map and `robot_params` with paths relative to the output file that resolve to real files; no absolute path or home directory in the text |
| `fsb_patrol` | `test_incident.py` (15) | Lying-person localisation from LiDAR (background and isolated points rejected, LiDAR mounting respected), viewpoint selection, "position is empty" decision (person gone / still there / occluded / outside FoV), watchlist merging and persistence |

## Level 2 — Planner (no Gazebo, part of `colcon test`)

`fsb_patrol/test/test_planner_variants.py` runs `patrol_planner` on 7 cases derived from the saved
factory map (original, rotated 90° and 30°, mirrored, two crops, mirror + 17°).

**Expected per case:** no unreachable waypoint pair, all waypoints valid (traversable, cost limit),
coverage ≥ 95 %, all corridor segments entered.

Manual run on the real map:
```bash
ros2 run fsb_patrol patrol_planner --map src/fsb_navigation/maps/factory.yaml \
    --robot-params src/fsb_patrol/config/robot_params.yaml --out /tmp/wp.yaml
```
**Expected / measured (2026-11-20):** 29 waypoints, coverage 95.02 % (narrow 93.7 %, wide 98.3 %),
12/12 corridor segments, min clearance 0.475 m, tour 88.3 m (grid costs).

## Level 3 — Perception (needs the venv; Gazebo optional)

Prerequisite: `scripts/setup_perception_venv.sh`.

```bash
PYTHONNOUSERSITE=1 ~/fsb_venv/bin/python3 -c \
  "import cv2, ultralytics; from cv_bridge import CvBridge; print(cv2.__version__)"   # 4.x (apt)
ros2 run fsb_gazebo posture_monitor.py     # per-posture frame counts + alerts (Ctrl+C → summary)
```

| Measurement | Result | Sample |
|---|---|---|
| Inference rate (yolov8n-pose, 640×480, CUDA) | ~5.4 Hz; 5.2–5.4 Hz from compressed input | — |
| Walking person during patrol, confidence ≥ 0.8 | 95.1 % of frames (median 0.891) | 453 frames |
| Lying-person capture | 12/12 found, 1 duplicate entry | 4 worlds × 3 people |
| Position error (first → after recording) | median 0.28 → 0.23 m, max 0.80 → 0.65 m | 12 people |
| Negative world (standing person + box, full loop) | 0 lying decisions, **0 alerts** | 4525 inferences |
| Fixed scene false "lying" | standing 0/512, box 0/192 frames | — |
| Single-frame "lying" vs. camera height | 4.7 cm: 55 %, 60 cm: 67 % | 2464 frames |
| `cmd_vel_watchdog` | zero velocity 0.502–0.519 s after commands stop | 10/10 trials |

## Level 4 — Simulation integration

Start the simulation in one terminal (`headless:=true` if no GUI is needed) and the measurement
script in another. Most scripts print a summary on Ctrl+C or with `--report N`. Scripts are in
`src/fsb_gazebo/scripts/`; usage is in each file's docstring.

| Test | Command (outline) | Expected | Measured |
|---|---|---|---|
| Wheel odometry | `bringup.launch.py` + `drive_test.py` | odom ≈ ground truth | 1 m forward: 1.0260 m odom = 1.0260 m true; ±360°: −0.003° / +0.002° |
| EKF drift | 5 min random driving (`teleop_test.py`) | closed-loop error < 2 cm / < 1° | 0.18 cm / +0.11° and 1.08 cm / −0.26° (~44 m, ~3000° turned) |
| LiDAR mounting | `scan_alignment_check.py` | correct mount ≫ +180° hypothesis | 97.9 % / 98.5 % inliers vs. 1.5 % / 5.7 % |
| Map quality | `localization:=slam` + `mapping_tour.py`, then `map_quality_check.py --map <map.yaml>` | no double walls, p95 surface error < 10 cm | median 2.2 cm, p95 5.5 cm, 99.96 % ≤ 10 cm, coverage 99.1 % |
| AMCL convergence | `localization:=amcl` + `amcl_check.py` | converges from a wrong initial pose | +60 cm / 16.5° → 1.2 cm / 0.5° in 7.0 sim-s |
| Global planner | SmacPlanner2D on all 810 waypoint pairs | no pair without a path | 0/810 (NavFn: 21/810 failed) |
| Dynamic obstacle | `dynamic_obstacle_test.py` (0.5 m box dropped on the path) | goal reached, no contact | reached, min clearance 38.8 cm |
| 4-point patrol | `patrol.launch.py loops:=10` | all loops | 10/10, 0 recoveries |
| Idleness patrol | `patrol:=true loops:=5` + `clearance_monitor.py`, `localization_monitor.py`, `recovery_counter.py` | no collision, no skip | 5/5 loops, 156 visits, 0 skipped, 0 recoveries, 0 collisions, min clearance 10.5 cm, `map→odom` jump max 4.71 cm (p99 1.87 cm) |
| Coverage from actual path | `coverage:=true` | ≥ 95 % | 99.95 % (narrow 99.94 %), 12/12 corridors |
| Real-time factor | `perf_monitor.py --duration 240` | — | 0.951 no perception · 0.944 YOLOv8n-pose (raw) · 0.910 YOLOv8n-pose (compressed + watchdog, 3 lying people) |
| **0.2 m/s limit** (2026-11-20) | `localization:=amcl headless:=true`, 4 `NavigateToPose` goals, then 0.3 m/s sent directly to `/cmd_vel` | command ≤ 0.2 m/s; controller clamps 0.3 | 4/4 SUCCEEDED; max `/cmd_vel` 0.200, EKF 0.203, controller odom 0.202 m/s; direct 0.3 m/s → odom **0.201 m/s** |
| **Short patrol at 0.2 m/s, clean incident log** (2026-11-20, after the smoother fix below) | `localization:=amcl headless:=true` + `patrol_executor.launch.py loops:=1 alert_log_dir:=<empty dir>` | loop completes, no timeouts | **Loop 1/1 complete**: 30/30 visits, 0 skipped, 0 timeouts, 0 revisits; 972 sim-s (1005 s wall); leg time median 26.6 / max 95.1 sim-s (limit 180); max `/cmd_vel` 0.200, controller odom 0.205, EKF 0.203 m/s; **0** "older than current time" warnings |
| Short patrol at 0.2 m/s, first attempt (2026-11-20, superseded) | `patrol_executor.launch.py loops:=1` with the default log dir | loop completes, no timeouts | Stopped by the 1700 s wall-clock limit before the loop finished: 25/29 waypoints reached, **0 skipped, 0 timeouts**, leg time median 25.9 / max 58.4 sim-s (limit 180); max `/cmd_vel` 0.200, controller odom 0.206, EKF 0.204 m/s. The loop was lengthened by 25 revisit viewpoints for two incident entries left in `~/.ros/fsb_security` by earlier runs. The controller logged 281 "message older than current time" warnings; cause found and fixed (see "Known issues") |

## Known issues and resolved findings

- **Resolved — stale `cmd_vel` stamps (2026-11-20).** `diff_drive_controller` logged
  "Ignoring the received message ... older than the current time" 281 times in one patrol. The
  warnings came in bursts of ~11 identical stamps: with
  `stamp_smoothed_velocity_with_smoothing_time: false`, `velocity_smoother` keeps the stamp of the
  last *input* command and, when its input stops (goal reached, controller paused), republishes it
  at 20 Hz until `velocity_timeout` (1.0 s). After 0.5 s (`cmd_vel_timeout`) those messages are too
  old and the controller drops them. Behaviour was safe (the controller stopped on its own timeout)
  but the log was noisy. Fix: `stamp_smoothed_velocity_with_smoothing_time: true` and
  `velocity_timeout: 0.5` (= `cmd_vel_timeout`); guarded by `test_speed_limits.py`. The repeat
  patrol logged 0 warnings.
- **Incident state persists across runs.** `~/.ros/fsb_security` is reloaded on start (by design,
  for a real robot). For reproducible measurements use an empty directory
  (`security_log_dir:=$(mktemp -d)` / `alert_log_dir:=...`).

## CI

`.github/workflows/ci.yml` runs Levels 1–2 in the `ros:jazzy` container: `rosdep install`
(Gazebo, RViz and GUI keys skipped), `colcon build`, `colcon test`. Nothing in these levels starts
Gazebo or needs a GPU. Levels 3–4 are not run in CI.

## Last local run

`scripts/ci.sh` on the development machine (2026-11-20): **123 tests, 0 errors, 0 failures,
2 skipped** (`ament_copyright` in `fsb_patrol` and `fsb_perception`). Counts are as reported by
`colcon test-result` (CTest entries plus individual pytest cases).

| Package | Count | Content | Result |
|---|---|---|---|
| `fsb_bringup` | 8 | lint | pass |
| `fsb_control` | 13 | lint + watchdog (3) | pass |
| `fsb_description` | 25 | lint + consistency (14) | pass |
| `fsb_gazebo` | 24 | lint | pass |
| `fsb_navigation` | 17 | lint + speed limits (6) | pass |
| `fsb_patrol` | 26 | lint + incident (15) + planner variants (7) + planner output (1) | pass, 1 skipped |
| `fsb_perception` | 10 | lint + posture (7) | pass, 1 skipped |
