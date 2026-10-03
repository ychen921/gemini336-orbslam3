# Gemini 336 ORB-SLAM3

Run these commands inside the Docker container. Start the camera or rosbag in a
separate terminal with the same `ROS_DOMAIN_ID`. The viewer requires `DISPLAY`
and X11 access configured in the container.

## Build package

```bash
cd /workspaces/gemini336-orbslam3
source /opt/ros/humble/setup.bash
colcon build --packages-select gemini336_orbslam3 --symlink-install
source install/setup.bash
```

## Stereo + viewer

```bash
bash scripts/run_slam.sh \
  -p sensor_mode:=stereo \
  -p settings_path:=/workspaces/gemini336-orbslam3/configs/Gemini_336_stereo.yaml \
  -p enable_viewer:=true
```

## Stereo IMU + viewer

```bash
bash scripts/run_slam.sh \
  -p enable_viewer:=true
```

The launcher sources `install/setup.bash`, defaults to Stereo IMU, and saves logs
under `log_slam/`.

## Logging

Project spdlog messages appear in the terminal and
`log_slam/<session>/slam.log`. Both use `logging.level` (default: `info`), set at
startup. Supported levels: `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`.

```bash
bash scripts/run_slam.sh -p logging.level:=debug
```

`log_slam/latest` points to the latest session. The launcher also captures all
stdout/stderr in `log_slam/console_*.log`, including RCLCPP and ORB-SLAM3 output;
ORB-SLAM3 internal messages are not included in `slam.log`.

To change the launcher's log directory, set an absolute path inside the container:

```bash
GEMINI336_SLAM_LOG_DIR=/tmp/gemini336-logs bash scripts/run_slam.sh
```

### Logging in code

Get a module logger from the existing `LoggingSession`:

```cpp
#include "gemini336_orbslam3/logging.hpp"

// logging_session is the existing session owned by the caller.
const auto logger = logging_session.GetLogger("imu_frontend");
logger->info("event=START mode={}", "stereo_imu");
logger->warn("event=IMU_GAP gap_sec={}", gap_sec);
logger->error("event=TRACK_FAILURE frame_id={}", frame_id);
```

Use nonempty module names containing letters, digits, `_`, `-`, or `.`.
Stop and join all producers before finishing or destroying the session; do not
use loggers afterwards. Prefer state changes, anomalies, and summaries over
per-sample logging.

See [Logging infrastructure](src/gemini336_orbslam3/README_logging.md) for async
queue behavior, flushing, directory configuration, and lifecycle details.
