#!/bin/bash
echo "start recording stereo camera and IMU data to bag file"
ros2 bag record \
  /camera/gyro_accel/sample \
  /camera/left_ir/camera_info \
  /camera/right_ir/camera_info \
  /camera/left_ir/image_raw \
  /camera/right_ir/image_raw \
  /tf \
  /tf_static