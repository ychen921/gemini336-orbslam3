# Stereo-IMU coordination validation

This standalone C++17 test build uses the real SlamNode implementation and real
Stereo/IMU frontends, replacing only OrbSlam3Adapter with an observable backend
stub. No ORB-SLAM3 vocabulary, calibration or hardware is needed. It does not
validate actual inertial initialization, backend reset behavior or tracking accuracy.

CMake generates test-only copies exposing private state and renaming the node's
`main`. Production headers and interfaces are unchanged. The state tests directly
invoke callbacks to test boundary cases deterministically; the Python integration
suite separately exercises real ROS subscriptions, the executor, timer, production
main, exit codes and SIGINT. The IMU frontend itself is not stubbed.

Run inside `gemini336-orbslam3:latest`, with the repository mounted at `/workspace`
and a writable results directory at `/validation`:

```bash
source /opt/ros/humble/setup.bash
cmake -S /workspace/src/gemini336_orbslam3/tools/coordination \
  -B /validation/gemini336-6b3c-tools-build
cmake --build /validation/gemini336-6b3c-tools-build -j2
ROS_LOCALHOST_ONLY=1 /validation/gemini336-6b3c-tools-build/coordination_test
mkdir -p /validation/gemini336-6b3c-ros
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 \
  /workspace/src/gemini336_orbslam3/tools/coordination/ros_integration.py \
  /validation/gemini336-6b3c-tools-build/coordination_test \
  /validation/gemini336-6b3c-ros
```

Reconfigure after changing the node or IMU header so generated copies are current.
The test project is intentionally independent of the production colcon build.
Python uses the image's existing rclpy; no dependencies are installed.

Coverage includes startup coverage/wait/recovery, exact batch boundaries, normal
single-frame processing, data gaps, necessary-history overflow versus harmless
old-sample eviction, backwards IMU with no pending images, queue capacity,
startup and per-frame deadlines, invalid requests, invalid image times, parameter
rejection, backend exceptions after the first successful frame, accounting and
shutdown while waiting. Integration cases cover Stereo and Stereo-IMU, gaps,
backwards timestamps, timeout, backend failure and SIGINT.

`enqueue_to_return_*` measures from wrapper enqueue to normal backend return;
it excludes sensor/transport latency. Tracking state `NotInitialized` is used by
the stub intentionally: normal calls must be counted even before tracking is Ok.
ROS tests wait for discovery and its teardown between processes; they are functional
tests, not throughput benchmarks or evidence of lossless real-world input.

## Validation result (2026-09-22)

- Production package: Docker `colcon build --packages-select gemini336_orbslam3
  --symlink-install` passed.
- Direct-state suite: 61 assertions passed.
- ROS integration: all seven cases passed. Stereo, Stereo-IMU and SIGINT returned
  0; gap, backwards IMU, waiting timeout and injected backend failure returned 1.
  Every case logged one completed backend shutdown.
- Normal Stereo-IMU case: 10 enqueued, 10 processed, 0 discarded, 0 pending;
  backend-failure case: 2 enqueued, 1 processed, 1 pending;
  SIGINT-while-waiting case: 10 enqueued, 0 processed, 10 pending.
- An initial ROS run exposed publisher discovery overlap between sequential
  processes; the fixture now waits for subscription teardown and discovery settling.
  The final seven-case run passed with this fixture fix.
- Raw logs are temporary host artifacts in `/tmp/gemini336-6b3c-test.log` and
  `/tmp/gemini336-6b3c-ros/`; committed test sources support rerunning them.
- No real backend, bag, viewer, sensor hardware, sustained throughput or end-to-end
  latency validation was performed. The backend stub cannot certify actual
  ORB-SLAM3 resource cleanup or safe behavior after an internal map reset.
