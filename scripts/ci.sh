#!/usr/bin/env bash
# Build + test steps shared by GitHub Actions (.github/workflows/ci.yml) and local runs, so the local
# check is the same command sequence as CI. Run from the workspace root after dependencies are
# installed (scripts/install_deps.sh). No test here starts Gazebo or needs a GPU.
#
# Usage: scripts/ci.sh
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
set +u; source /opt/ros/jazzy/setup.bash; set -u

colcon build --event-handlers console_cohesion+          # no --symlink-install (project rule)
colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
colcon test-result --all --verbose
