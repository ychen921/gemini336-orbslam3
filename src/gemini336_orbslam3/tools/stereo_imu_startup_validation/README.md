# 6C-3：真實 Stereo-IMU backend 無輸入啟動驗證

日期：2026-09-23。受測版本：`f396d1fbac02e1267fe487c34cdfd077b8088c90`。

## 結論與範圍

真實 ORB-SLAM3 Stereo-Inertial 設定載入、ROS 接線、無輸入等待與 SIGINT 程序退出通過。
沒有啟動 camera driver、bag player 或 viewer，沒有送入影像或 IMU。
因此不代表慣性初始化、tracking、map reset、精度、持續效能或硬體驗收通過。

本輪正式程式與設定未修改，僅新增驗證紀錄與重跑腳本。

## 建置

使用 `gemini336-orbslam3:latest`，以下命令退出碼 0，1 package finished：

```bash
CONTAINER_NAME=gemini336-viewer-build CONTAINER_HOME=/tmp/gemini336-viewer-home \
  bash docker/run_ubuntu.sh colcon build \
  --packages-select gemini336_orbslam3 --symlink-install
```

已確認 install 下的 `stereo_imu_slam.yaml` 與 node executable 存在。
實際啟動 install 下的 executable，直接記錄程序退出碼；未使用 tee 管線。
完整 argv 與參數檔內容見 [command.json](final/command.json)。

## 最終驗證結果

| 項目 | 結果 |
|---|---|
| Backend | 日誌為 `Stereo-Inertial`，vocabulary 載入成功 |
| Legacy settings | 正常讀取，沒有缺欄位／型別錯誤 |
| Tbc | camera → body；平移約 `[-0.000246, -0.000065, 0.016948] m`，旋轉為 identity |
| IMU frequency | 200 Hz |
| NoiseGyro / NoiseAcc | `3.2e-05` / `0.0004` |
| GyroWalk / AccWalk | `2e-06` / `8e-05` |
| 參數快照 | ROS 參數檔的 16 個值及型別全部與 node 查詢結果一致 |
| Sensor topics | 左右 IR 與 `/camera/gyro_accel/sample` 各 1 subscriber、0 publishers |
| QoS | 三個 sensor subscriptions 皆 BEST_EFFORT、VOLATILE；CLI 的 History (Depth) 為 UNKNOWN，不宣稱已由 graph 驗證深度 |
| 啟動時間 | 腳本輪詢觀察約 4.01 秒，非精確效能量測 |
| 無輸入等待 | 發現初始化日誌後持續存活約 6.23 秒，超過設定的 5 秒影像 timeout |
| 處理數量 | IMU received/accepted=0；enqueued/processed/pending=0 |
| SIGINT | 約 0.214 秒內程序退出，退出碼 0，未強制終止 |
| 最終輸出 | total tracking report、final coordination report、shutdown returned 各 1 次 |

證據：[node.log](final/node.log)、[parameters.yaml](final/parameters.yaml)、
[node_info.txt](final/node_info.txt)、[left_topic.txt](final/left_topic.txt)、
[right_topic.txt](final/right_topic.txt)、[imu_topic.txt](final/imu_topic.txt)、
[result.json](final/result.json)。

### 保留限制

- 上游 shutdown 印出 `mpLocalMapper is not finished`、`mpLoopCloser is not finished`，
  隨後返回且程序退出。僅確認本次程序退出，不能據此宣稱內部工作執行緒已完整 join 或資源全部回收。
- 四個噪聲值成功載入不代表 SDK 與 ORB-SLAM3 噪聲模型／單位映射已被物理驗證；沿用 6C-1/2 的假設。
- 無輸入測試驗證 timeout 尚未啟動的契約，不驗證有資料後的 timer 精度或取樣能力。

## 首輪觀察與重跑原因

首輪使用 Docker 預設 shared-memory 容量，node 正常啟動、等待及退出，
但部分 ROS CLI 查詢輸出含 Fast DDS SHM segment 建立失敗訊息。
保留 [初輪快照](initial/parameters.yaml)、[node 日誌](initial/node.log) 與
[結果](initial/result.json)，不把初輪含診斷文字的參數輸出當成乾淨 YAML。

最終輪改用 `--shm-size=256m`，保留同一專案 Fast DDS profile；查詢與 node 日誌未再出現上述錯誤。
此變更只適用本次隔離驗證容器，未修改專案 Docker 設定。

## 重跑

在 host 建立可寫的輸出目錄並複製腳本（輸出檔案會被覆寫）：

```bash
mkdir -p /tmp/gemini336-6c3-replay
cp src/gemini336_orbslam3/tools/stereo_imu_startup_validation/run.py /tmp/gemini336-6c3-replay/
docker run --rm --network none --shm-size=256m \
  -e ROS_DOMAIN_ID=91 -e ROS_LOCALHOST_ONLY=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -v "$PWD:/workspaces/gemini336-orbslam3:ro" \
  -v /tmp/gemini336-6c3-replay:/validation \
  --entrypoint bash gemini336-orbslam3:latest -c '
    source /opt/ros/humble/setup.bash &&
    source /workspaces/gemini336-orbslam3/install/setup.bash &&
    export LD_LIBRARY_PATH=/workspaces/gemini336-orbslam3/external/install/pangolin/lib:$LD_LIBRARY_PATH &&
    python3 /validation/run.py'
```

需從 repository root 執行，並先完成上述建置。網路隔離搭配獨立 domain；
檢查腳本要求三個 topics 均無 publisher，避免把意外輸入混入驗證。
