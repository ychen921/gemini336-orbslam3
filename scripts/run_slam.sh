#!/usr/bin/env bash

# Run inside the development container; forward arguments as ROS arguments.
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
source "$PROJECT_ROOT/install/setup.bash"

# Keep the existing SLAM session/latest layout. A unique console file also
# captures ros2run's own exit diagnostics after the node has terminated.
LOG_ROOT="${GEMINI336_SLAM_LOG_DIR:-$PROJECT_ROOT/log_slam}"
mkdir -p "$LOG_ROOT"
CONSOLE_LOG="$(mktemp "$LOG_ROOT/console_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")"
echo "Console log: $CONSOLE_LOG"

# pipefail preserves a failed ros2 run status even when tee succeeds.
# The explicit directory also keeps console and session logs under one root.
ros2 run gemini336_orbslam3 slam_node --ros-args \
  --params-file "$PROJECT_ROOT/src/gemini336_orbslam3/config/stereo_imu_slam.yaml" \
  -p "logging.directory:=$LOG_ROOT" \
  "$@" 2>&1 | tee "$CONSOLE_LOG"
