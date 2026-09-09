#!/usr/bin/env bash

set -e

ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_depth:=false \
  enable_laser:=false \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_sync_output_accel_gyro:=true
