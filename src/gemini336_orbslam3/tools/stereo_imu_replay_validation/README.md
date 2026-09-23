# 6D-1：短片段 Stereo-IMU 真實 backend 回放

2026-09-23 執行，受測 commit `caccfc22b495c38e03e787e6abe77c4313826404`。
本輪新增驗證工具與紀錄，未修改正式 C++、settings、ROS 參數或第三方程式。

## 結論

約 60 秒、1.0× 左右影像＋合併 IMU 回放通過本輪整合檢查；
沿用有限序列最後一對不輸出的已知限制。觀察到 VIBA 1、2 與後續持續 tracking，
支持真實慣性初始化曾完成，不代表校正、尺度或定位精度已驗證。
完整 bag、Stereo 回歸、實機及長時間驗收尚未執行。

## 設定與片段

Docker `gemini336-orbslam3:latest`，image ID、建置與工作目錄狀態見
[provenance.json](final/provenance.json)。Docker C++17 套件建置通過，退出碼 0。
測試 network none、ROS domain 92、Fast DDS、shared memory 256 MiB，viewer 關閉。
沿用 `stereo_imu_slam.yaml`，僅增加 `slam_node:=debug` 日誌等級。

來源 `bags/test_gemini336_stereo_imu`；以第一張左影像 header timestamp 為起點，
取不晚於起點＋60 秒的左影像及同序右影像，逐對確認時間差不超過 2 ms。
保留 bag 起始 IMU 至最後左右影像之後的第一筆 IMU。
使用新 rosbag 保存原始 serialized bytes、recorded timestamps 與錄製順序，
不修改 header timestamp，不追加合成訊息，不暫停或循環。
現有 Humble player 不支援 duration，因此先產生確定邊界的片段，再自然回放至結束。

| 項目 | 結果 |
|---|---:|
| 左影像 header 範圍 | 1789957367462902000～1789957427435031000 ns |
| 影像時間跨度 | 59.972129 秒 |
| 左／右影像 | 各 1,798 張 |
| IMU | 11,895 筆 |
| IMU 首端覆蓋 | 比第一張左影像早 17.162 ms |
| IMU 尾端覆蓋 | 比最後一張左影像晚 4.664 ms |

完整邊界見 [clip_manifest.json](final/clip_manifest.json)，逐筆原始時間戳見
[clip_timestamps.json](final/clip_timestamps.json)。產生的 clip 約 1.5 GB，僅保留於
host `/tmp/gemini336-6d1/clip`，不納入 repository；可由原始 bag 重建。

## 驗證結果

| 項目 | 結果 |
|---|---|
| 左右原始接收 | 各 1,798 張，逐筆整數 timestamp 與片段完全一致 |
| 同步 | 1,797 組，精確對應來源第 1～1,797 對，只有尾端一對未輸出 |
| Tracking | 1,797 次正常返回；130 幀 NotInitialized，1,667 幀 Ok |
| 初始化證據 | start/end VIBA 1、start/end VIBA 2 均出現 |
| VIBA 2 後 | 1,176 幀皆 Ok，未觀察到 reset 日誌 |
| 啟動運動條件 | 128 次 not enough acceleration，之後進入 tracking 與 VIBA；未為此改設定 |
| IMU | received=accepted=11,895，backwards=overflow=0，結束 buffered=9 |
| 協調 | enqueued=processed=1,797；startup_discarded=pending=0；pending_peak=5 |
| Tracking 耗時 | mean 14.843 ms，max 31.211 ms |
| 入列至 backend 返回 | mean 17.270 ms，max 63.414 ms；不含上游延遲 |
| Timestamp 數值 | Tracking 秒制輸出對原始 ns 最大差 239 ns，符合此 epoch 的浮點表示量級 |
| 自然退出 | 最後影像後 idle=5.039 秒觸發既有 shutdown；player/node 退出碼均 0 |
| 清理紀錄 | 最終統計及 shutdown returned 各一次，無 SIGINT 或強制終止 |

每個定期協調快照的 pending 均為 0，峰值為 5；未見持續增長。
`scope=total` 的約 24.99 Hz 包含啟動等待與尾端靜默，不能當作回放期間吞吐率。
受測影像時間間隔 mean 約 33.373 ms。

證據：[node.log](final/node.log)、[player.log](final/player.log)、
[result.json](final/result.json)、[analysis.json](final/analysis.json)、
[逐幀紀錄](final/frames.json)、[實際參數](final/parameters.yaml)。
`result.json` 為執行器基礎檢查，`analysis.json` 額外核對原始接收、同步與初始化後狀態。
兩者 passed 均為 true，僅表示上述範圍通過。

## 證據限制

- 未新增 per-batch IMU trace 或初始化狀態 API；首張空 batch、後續逐批內容
  沿用已完成的 6B 接線／合成證據，本輪不宣稱獨立逐筆驗證 backend 收到的 IMU 批次。
- 本地 `LocalMapping.cc` 的 VIBA 分支要求 IMU initialized 且 tracking OK，
  因此 VIBA 日誌是初始化曾完成的間接證據；start/end 不能證明最佳化精度。
- 第一段 NotInitialized 與運動不足是 backend 狀態，不是 wrapper startup discard。
- 同步器內部尾端仍有一對未輸出；SlamNode pending=0 不等於所有原始影像均處理。
- 上游退出仍印出 LocalMapper／LoopCloser not finished；程序退出成功不代表所有執行緒完整 join。
- DEBUG 日誌增加負載；本輪耗時僅作觀察，不是端到端延遲或效能認證。
- 沿用 factory 外參／噪聲映射假設與 20 ms gap 門檻，未證明可容忍 20 ms 資料遺失。

## 重跑

先建置套件。從 repository root 執行以下命令，輸出目錄必須是新的；
工具拒絕覆寫既有 clip 或 result.json。需保留原始 bag，無需新增套件。

```bash
mkdir -p /tmp/gemini336-6d1-rerun
docker run --rm --network none --shm-size=256m \
  -e ROS_DOMAIN_ID=92 -e ROS_LOCALHOST_ONLY=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -v "$PWD:/workspaces/gemini336-orbslam3:ro" \
  -v /tmp/gemini336-6d1-rerun:/validation \
  --entrypoint bash gemini336-orbslam3:latest -c '
    source /opt/ros/humble/setup.bash &&
    source /workspaces/gemini336-orbslam3/install/setup.bash &&
    export LD_LIBRARY_PATH=/workspaces/gemini336-orbslam3/external/install/pangolin/lib:$LD_LIBRARY_PATH &&
    python3 /workspaces/gemini336-orbslam3/src/gemini336_orbslam3/tools/stereo_imu_replay_validation/run.py'
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/analyze.py \
  /tmp/gemini336-6d1-rerun
```

執行器先確認 node 就緒且 sensor topics 無其他 publisher，再給 player 2 秒 discovery 延遲。
Startup watchdog 60 秒、player watchdog 100 秒、player 結束後 node 退出上限 30 秒。
異常清理先 SIGINT，10 秒未退出再 kill；需要清理的執行不判為自然退出通過。
重新執行時應另保存當下 git/image 版本及建置結果；provenance.json 為本輪 host 記錄。
