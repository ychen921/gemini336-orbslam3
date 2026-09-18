#!/usr/bin/env bash

set -e

source install/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_color:=true \
  color_width:=848 \
  color_height:=480 \
  enable_depth:=true \
  depth_width:=848 \
  depth_height:=480 \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  enable_frame_sync:=true \
  frame_aggregate_mode:=full_frame \
  enable_laser:=true \
  enable_left_ir:=true \
  enable_right_ir:=true \
  enable_sync_output_accel_gyro:=true