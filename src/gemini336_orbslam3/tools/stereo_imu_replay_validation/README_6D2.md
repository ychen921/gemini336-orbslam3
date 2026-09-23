# 6D-2：完整 Stereo-IMU bag 原速回放

2026-09-23 執行。**本輪未通過：回放中途發生 IMU DataGap，完整 bag 未跑完。**
保留失敗結果，不以停止前 tracking 為 Ok、player 退出碼為 0 或 pending 峰值小
改判通過。未執行降速替代驗收、參數調整、架構修改或 6D-3。

## 範圍與設定

使用 `gemini336-orbslam3:latest`，network none、ROS domain 93、shared memory 256 MiB，
Fast DDS，viewer 關閉。Docker 套件建置退出碼 0，1 package finished [0.52s]。
版本、工具差異與 image ID 見 [provenance.json](final/full_bag/provenance.json)。
正式 C++、backend settings 與 ROS 參數均未修改；只增加驗證工具的 `--full-bag` 模式。

直接讀取原始 `bags/test_gemini336_stereo_imu`，以 1.0× 播放左右 IR 與合併 IMU，
不裁切、不修改訊息、不新增影像、不循環。來源左右各 9,123 張、IMU 60,364 筆，
影像 header 時間跨度 304.428784 秒。完整 IMU 尾端也包含在播放範圍。
沿用 2 ms stereo 容差、20 ms IMU gap、IMU QoS depth 200、buffer 2000、
pending capacity 30、wait timeout 1 秒，並啟用 `slam_node:=debug`。

## 結果

| 項目 | 本輪觀察 |
|---|---|
| 原始影像接收 | 左右各 4,695 張；最後接收到來源第 4,794 張 |
| 接收缺口 | 左右在最後已接收 timestamp 之前各缺 99 張，包含啟動與中段缺口 |
| 同步與 tracking | 同步／入列 4,693 組，正常返回 4,688 次，最終 pending 5 |
| Accounting | 4,693 = 4,688 processed + 0 startup discard + 5 pending |
| 已處理 header 跨度 | 約 156.690 秒；最後完成來源第 4,696 張 |
| Tracking state | 128 幀 NotInitialized、4,560 幀 Ok；未觀察到 RecentlyLost／Lost |
| 初始化與 map | VIBA 1／2 均 start/end；VIBA 2 後 4,080 次返回皆 Ok；未見 reset |
| Loop 日誌 | 兩次 Loop detected，其中一次 BAD LOOP；不能僅據此宣稱成功閉環或精度 |
| IMU | received=accepted=31,273，backwards=overflow=0，buffered=206 |
| Tracking 耗時 | mean 18.508 ms，P95 25.149 ms，max 2,131.390 ms |
| 入列至返回 | mean 21.329 ms，max 2,139.737 ms；不涵蓋 DDS／上游等待 |
| Pending | 峰值 6；多數定期快照為 0，失敗時為 5 |
| Node | DataGap 錯誤後退出碼 1；最終統計與 shutdown returned 各一次 |
| Player | node 提前退出後由驗證工具 SIGINT 停止，退出碼 0；不是自然播完 |

觀測的狀態區間依影像 timestamp 計算：NotInitialized 約 4.339 秒、Ok 約
152.351 秒。每幀狀態歸屬到下一幀時間，最後一幀不外推；含來源接收缺口，
不等於該段所有影像都被處理或零失追認證。

## 失敗定位

最後正常返回後，下一個取樣請求報錯：

```text
IMU data gap detected:
interval=(1789957524.1524239, 1789957527.2551229] pending=5
```

- 原始 bag 在這個區間包含必要左右 bracketing samples，共 617 筆（含邊界參考），
  最大 IMU 間隔 **5.057 ms**；全 bag 最大 **5.087 ms**，皆低於 20 ms。
  因此原始資料的該段時間間隔無法解釋 runtime DataGap。
- 左右原始 callback 日誌各缺 99 張。左右皆缺來源第 3、4、3,247～3,251、4,605 張，
  後續第 4,697～4,788 張間又有大量缺口，兩側少數接收位置不一致。
  完整索引見 `analysis.json` 的 `raw_reception_gaps`。
- 第 3,243 次 tracking 呼叫耗時 333.941 ms；第 4,688 次耗時 2,131.390 ms。
  後者附近原始 callback 的最大影像 header 間隔達左 2,536.386 ms、右 2,469.639 ms。
- 有些 callback 延遲已發生在最後一次長 tracking 呼叫之前，不能把全部缺口
  簡化歸因為該次呼叫；也不能把它直接歸因為先前 Loop detected。
- 現有單執行緒在 tracking 期間無法處理其他 ROS callback；IMU 訂閱使用有界
  KEEP_LAST 200，約相當於一秒 IMU。這支持「長時間處理或系統停頓造成接收不足」
  的調查方向，但本輪沒有 middleware 丟棄統計、逐筆 runtime IMU timestamps、
  CPU／memory／scheduler 或 backend lock trace，**尚未確定根因**。
- `overflow=0` 只表示 ImuFrontend 自有 buffer 未淘汰，不表示 DDS 沒有遺失。
  同理 pending 小只反映已到 callback 的影像，不能證明上游沒有積壓或丟棄。

錯誤日誌到最終統計相隔約 16.5 秒，之後 shutdown 返回；沒有 watchdog 強制 kill。
此為錯誤路徑退出，不能算正常自然退出驗收通過，也尚未定位清理延遲來源。

## 證據與檢查

- [完整 node 日誌](final/full_bag/node.log)、[player 日誌](final/full_bag/player.log)
- [執行結果](final/full_bag/result.json)、[離線交叉核對](final/full_bag/analysis.json)
- [來源清單](final/full_bag/clip_manifest.json)、[原始時間戳](final/full_bag/clip_timestamps.json)
- [參數快照](final/full_bag/parameters.yaml)、[實際命令](final/full_bag/command.json)

`result.json`：10 項檢查中 6 項通過、4 項失敗。
`analysis.json`：9 項檢查中 3 項通過、6 項失敗。
所有 `passed=false` 保留。同步數與 processed 不相等是 5 筆尚未完成的 pending，
不是分析器應忽略的項目。`maximum_timestamp_error_ns` 在本輪為與來源同序位置
比較的差值，受到接收缺幀影響，不能當成浮點 timestamp 精度誤差。

分析工具也以 6D-1 歷史證據副本回歸，9 項檢查仍通過；沒有重跑短片段。
Python 語法與 `git diff --check` 通過。未驗證硬體、完整退出、完整後半段或 Stereo 回歸。

Artifacts 存於已被 `.gitignore` 忽略的 `final/full_bag/`，不納入 commit；
host 暫存副本為 `/tmp/gemini336-6d2/`。上述相對證據連結只在保留產物的本機有效。

## 重跑

先建置套件，從 repository root 執行，輸出目錄必須是新的：

```bash
mkdir -p /tmp/gemini336-6d2-rerun
docker run --rm --network none --shm-size=256m \
  -e ROS_DOMAIN_ID=93 -e ROS_LOCALHOST_ONLY=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml \
  -v "$PWD:/workspaces/gemini336-orbslam3:ro" \
  -v /tmp/gemini336-6d2-rerun:/validation \
  --entrypoint bash gemini336-orbslam3:latest -c '
    source /opt/ros/humble/setup.bash &&
    source /workspaces/gemini336-orbslam3/install/setup.bash &&
    export LD_LIBRARY_PATH=/workspaces/gemini336-orbslam3/external/install/pangolin/lib:$LD_LIBRARY_PATH &&
    python3 /workspaces/gemini336-orbslam3/src/gemini336_orbslam3/tools/stereo_imu_replay_validation/run.py --full-bag'
# Even when replay fails, run the analyzer to retain diagnostic evidence.
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/analyze.py \
  /tmp/gemini336-6d2-rerun
```

原始 bag 直接唯讀播放，不產生另一份完整 bag。沿用 `clip_*` 檔名作分析介面，
manifest 的 `full_bag=true` 區別此次完整來源。
Startup watchdog 60 秒，player watchdog 為影像跨度＋40 秒（本次約 344.429 秒），
player 自然结束後等待 node 至多 30 秒。錯誤清理先 SIGINT，10 秒未退出才 kill。

## 下一步建議（尚未執行）

先規劃接收缺口診斷：記錄 runtime IMU gap endpoints、DDS 接收遺失及系統負載，
並區分 backend 耗時、執行緒排程與傳輸影響。若以降速重跑作對照，僅能作原因診斷，
不能取代 1.0× 驗收。證據充分後再審閱接收／tracking 執行緒調整；
本輪不直接增大 buffer、放寬 gap 門檻或修改正式架構。
