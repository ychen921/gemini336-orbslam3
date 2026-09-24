# Logging infrastructure

Each `slam_node` construction creates `<workspace>/log_slam/YYYYMMDD_HHMMSS_PID/slam.log`.
A collision adds a numeric suffix. `log_slam/latest` atomically points to the newest
successfully initialized session. The default workspace is resolved at CMake configure
time; it does not depend on the node's current directory. In Docker the default is
`/workspaces/gemini336-orbslam3/log_slam`, on the mounted host workspace.

Startup-only ROS parameters:

- `logging.directory`: absolute directory; empty selects `GEMINI336_SLAM_LOG_DIR`,
  then the build-time workspace's `log_slam`. Override when relocating an installation.
- `logging.level`: `trace`, `debug`, `info` (default), `warn`, `error`, `critical`, `off`.

Invalid configuration or session initialization failures propagate to the existing
node startup error handler. The Docker launcher does not forward the logging environment
variable from the host; set it inside the container or use the ROS parameter.

`LoggingSession` is a ROS-independent library. Its cached module loggers share one
file sink and a private asynchronous queue (8192 entries, one writer). Queue overload
replaces the oldest record instead of blocking producers. `dropped_messages()` exposes
that count; shutdown reports a nonzero count to stderr. Records include local wall time
with UTC offset, producer thread ID, module, level, and the caller's message.

The sink flushes every second. Stop and join all producers before destroying the
session; destruction drains the queue and flushes the sink. Logger handles must not
be used afterwards. There is no process-global logger registry or global shutdown.
Abrupt process termination can lose queued/buffered records. Files do not rotate yet.

This change only initializes and owns the backend: it adds no frontend, synchronization,
or ORB-SLAM3 instrumentation and does not redirect existing RCLCPP/stdout output.
An unused session's `slam.log` is therefore empty. Instrumentation is a separate step.

The standalone `logging_test` checks concurrent module writes, filtering, shutdown
draining, unique sessions, latest symlink replacement and invalid configuration without
starting ROS nodes or accessing hardware.
