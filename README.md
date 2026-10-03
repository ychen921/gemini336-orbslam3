# Gemini 336 ORB-SLAM3

以下指令在 Docker container 內執行。相機或 rosbag 請在另一個 terminal 啟動，並使用相同的 ROS_DOMAIN_ID。Viewer 需要 container 已配置 DISPLAY 與 X11 存取。

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

腳本會自動載入 `install/setup.bash`，預設使用 Stereo IMU 設定；日誌存於 `log_slam/`。
