# 第 3 步：Viewer 開啟的原速診斷對照

使用第 2 步診斷工具，新增 `run.py --viewer` 選項，僅以 CLI 覆寫
`enable_viewer:=true`，不修改正式設定檔、queue、gap 門檻或執行緒架構。
保留 trace 與每秒系統負載採樣，輸出至新的目錄。

## 測試基準

- 同一份完整 `bags/test_gemini336_stereo_imu`，1.0×，三個 sensor topics。
- 同一 Docker image、network none、256 MiB shared memory、ROS domain 95。
- `parameters.yaml` 實際查詢比對第 2 步，唯一差異為 `enable_viewer`；
  backend settings 逐 byte 相同。
- 為存取 X11，容器使用 host UID/GID 1001:1001、DISPLAY=:1，唯讀掛載
  X11 socket 與 Xauthority。未改 xhost 權限、未掛載相機。
- 使用既有已建置 executable，保存 binary SHA-256；本輪只改 Python，無 C++ 修改。
- X11 與執行者身份是開啟 GUI 所需環境差異；系統當時負載、GPU renderer
  未固定，不能把單次結果差異全部歸因於 viewer。

## 重跑

從 repository root 執行；每次使用新目錄。下例 UID、DISPLAY 與 Xauthority
適用本次工作站，其他 session 應依實際環境調整；不需要輸出 Xauthority 內容。

```bash
mkdir -p /tmp/gemini336-step3-viewer-rerun
docker run --rm --network none --shm-size=256m --user 1001:1001 \
  -e HOME=/tmp -e DISPLAY=:1 -e XAUTHORITY=/tmp/viewer.xauth \
  -e ROS_DOMAIN_ID=95 -e ROS_LOCALHOST_ONLY=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /run/user/1001/gdm/Xauthority:/tmp/viewer.xauth:ro \
  -v "$PWD:/workspaces/gemini336-orbslam3:ro" \
  -v /tmp/gemini336-step3-viewer-rerun:/validation \
  --entrypoint bash gemini336-orbslam3:latest -c '
    source /opt/ros/humble/setup.bash &&
    source /workspaces/gemini336-orbslam3/install/setup.bash &&
    export LD_LIBRARY_PATH=/workspaces/gemini336-orbslam3/external/install/pangolin/lib:$LD_LIBRARY_PATH &&
    python3 /workspaces/gemini336-orbslam3/src/gemini336_orbslam3/tools/stereo_imu_replay_validation/run.py --full-bag --diagnostics --viewer'
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/analyze_trace.py \
  /tmp/gemini336-step3-viewer-rerun
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/analyze.py \
  /tmp/gemini336-step3-viewer-rerun
```

回放期間不暫停、不操作 viewer 的 Stop／Reset，以免混入不同測試條件。
Artifacts 保留於已忽略的 `final/viewer_diagnostics/`，不納入 commit。

## 本輪結果（2026-09-23）

**未通過：重現持續積壓，因影像等待期限停止；退出時另發生 heap corruption。**
不是本輪 IMU DataGap，也不能把第 2 步的單次成功當成問題已解決。

| 項目 | 第 2 步，viewer 關閉 | 第 3 步，viewer 開啟 |
|---|---:|---:|
| Tracking 正常返回 | 9,122 | 7,291 |
| 最終 pending／峰值 | 0／12 | 27／28 |
| IMU received=accepted | 60,364 | 48,302 |
| Callback 已接收範圍內 IMU 缺失 | 0 | 0 |
| 237～242 秒 tracking 平均 | 30.594 ms | 36.832 ms |
| 全輪 tracking 最大 | 54.961 ms | 70.340 ms |
| 結果 | 自然退出 0 | 等待逾時，後續 SIGABRT（Popen returncode -6） |

本輪左右各接收 7,319 張，到來源第 7,322 張；兩側皆缺第 7,289、7,311、7,316 張。
入列 7,318、處理 7,291、startup discard 0、pending 27，accounting 成立。
所有已完成 tracking 的初始化後狀態皆 Ok（130 NotInitialized、7,161 Ok）；
VIBA 1/2 出現，未見 reset。這些均不代表完整回放通過。

原始報錯：

```text
Stereo frame wait timed out: waited_sec=1.00298 threshold_sec=1 pending=27
```

### 關鍵時間證據

第二次 loop 出現在 bag 約 232.816 秒附近，之後 pending 由 1、6、23 增至 27。
237～242 秒平均 tracking 36.832 ms，超過來源約 33.4 ms 幀週期；
242 秒至最後完成幀約 243.329 秒的 39 次呼叫平均 39.172 ms。
前一輪無 viewer 相同 237～242 秒為 30.594 ms。

尚未處理的最舊影像 timestamp 為 `1789957610.8252511` 秒：

| 事件 | steady clock ns |
|---|---:|
| 影像入列 | 205412495808210 |
| 左侧參考 IMU 1789957610824258000 ns 被接受 | 205412498384289 |
| 右側 IMU 1789957610829302000 ns 被接受 | 205412498391411 |
| 最後一筆 trace | 205413498718688 |

右側參考在入列後 **2.583 ms** 已接受，在最後 trace 前約 **1,000.327 ms** 就已存在。
IMU accepted timestamps 全段連續、沒有 reject／overflow 或 DataGap trace。
因此本次最舊影像已具備 IMU 覆蓋，逾時原因是前序影像處理造成排隊，
不是它的 IMU 還沒到。SlamNode 的期限从入列開始，且在取樣之前檢查。

此證據支持「現有串行流程在該區段處理量不足」；不等於確定 backend
內部的哪個計算／鎖／排程造成變慢，也不能用這一輪解釋先前每一次 IMU 缺口。
拆接收執行緒可能改善接收及短暫阻塞，但無法單獨解決長期 tracking 慢於輸入。

### 系統採樣與診斷完整性

- Trace 133,142 筆，無 dropped events；7 項完整性核對通過。
- IMU 48,302 筆全數對應來源前綴，無 unknown/reject/overflow/DDS lost event。
  未接收的 bag 後半段不計為遺失；零 DDS lost event 不能保證 middleware 零丟棄。
- 系統監測 248 筆，node 一秒 CPU 最大約 436%（4.36 cores）、player 約 23%；
  node RSS 最大 1,397,984 KiB。CPU 多核使用率不代表整台主機飽和。
- `result.json` 與 `analysis.json` 保留 failed；trace integrity 通過只代表證據完整。
- Player 在 node 提前結束後由工具 SIGINT 停止，退出碼 0 不等於自然播完。

### 獨立退出異常

等待逾時後，最終統計與 trace 都已保存。backend 印出多次 LocalMapper not finished，
`Stereo SLAM shutdown returned` 之後緊接：

```text
malloc(): unsorted double linked list corrupted
```

程序以 SIGABRT 結束。這是實際的 heap corruption 訊息，不能因已有 shutdown returned
就宣稱退出成功。沒有 backtrace／sanitizer 證據，目前不能定位是 backend、viewer、
診斷程式或銷毀時序造成；也不能由訊息位置推斷記憶體損壞首次發生時間。
先列為另一項退出缺陷，未修改第三方程式或自動追加測試。

## 產物與驗證

[回放結果](final/viewer_diagnostics/result.json)、[node 日誌](final/viewer_diagnostics/node.log)、
[trace 分析](final/viewer_diagnostics/trace_analysis.json)、
[前後比較](final/viewer_diagnostics/step2_comparison.json)、
[參數比對](final/viewer_diagnostics/parameter_comparison.json)、
[版本紀錄](final/viewer_diagnostics/provenance.json)。

本輪新增 `--viewer` 並更新說明；未修改正式 C++，因此沿用第 2 步已建置 binary。
Python 語法、git diff whitespace、參數唯一差異與 trace 計數交叉核對通過。
未驗證硬體、Stereo 回歸、長時間穩定性、記憶體錯誤根因或 keyframe pose 問題。

下一步可分開規劃：處理積壓／過載策略，以及退出時的記憶體與執行緒生命週期調查。
不建議只增加 queue 或延長 timeout 後宣稱通過；那只能延後過載停止。
