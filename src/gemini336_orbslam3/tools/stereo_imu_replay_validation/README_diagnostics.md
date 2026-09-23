# 第 2 步：接收與 tracking 時間診斷

## 實作範圍

沿用工作目錄已有的未提交診斷接線並補齊回放、系統監測及分析工具。
保留單執行緒與現有感測器參數；不增加 queue、不放寬 IMU gap，不處理 keyframe pose 異常。

- `diagnostics.trace_path`：預設空字串，關閉診斷；啟用時須為不存在的絕對路徑。
- `diagnostics.trace_capacity`：預設 300,000 個事件；啟用時一次 reserve，
  callback 內只寫入有界記憶體，不輸出 CSV、不逐筆動態配置。
- 記錄 IMU callback 收到的原始整數 ns、接受／拒絕原因、自有 buffer 淘汰、
  DDS message-lost event、取樣 DataGap 的請求與相鄰端點。
- 記錄左右原始 callback、frame 入列、tracking 呼叫前／返回後的共同
  `std::chrono::steady_clock` ns。Frame/query 邊界仍為原介面的 double 秒，
  不將它們稱為原始精確 ns；accepted trace 可找回對應來源整數時間戳。
- 拒絕事件接續在該筆 `imu_received` 之後，單執行緒順序用於關聯。
- Tracking 例外會留下沒有 end 的 begin；trace 分析器保留 unfinished_call。
- 停止 timers/subscriptions 後、backend shutdown 前才輸出 trace，
  CSV 尾端 `dropped_events` 回報超出容量而未記錄的事件數。
  若非零，不得把缺事件當成 sensor 遺失；寫檔失敗有 ROS error 且仍嘗試 backend cleanup。
- Trace 使用非 owning pointer，owner 壽命涵蓋兩個 frontends；只適用既有序列化
  executor。未加 mutex，日後多執行緒設計必須重新檢查。

`run.py --diagnostics` 啟用 trace，並在獨立驗證程序每秒讀取 `/proc`，
記錄 node/player CPU ticks、RSS、系統 CPU counters、MemInfo、load average。
資料先存記憶體，回放結束／錯誤清理後保存 `resources.json`。
Startup/discovery 與 cleanup 期間不在此每秒採樣迴圈內；看不到亞秒級資源變化。

## 重跑

沿用 README_6D2.md 的 Docker 命令與來源完整 bag，使用新的輸出目錄，
ROS domain 可改為 95，在 run.py 命令尾端加：

```text
--full-bag --diagnostics
```

容器內 `/validation` 須可寫。完成或失敗後，在 host 執行：

```bash
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/analyze_trace.py \
  /tmp/gemini336-step2
```

輸出 `trace_analysis.json`、`resource_metrics.json`。原有 `result.json` 保留
回放驗收結果；診斷成功取得證據不等於回放驗收通過。

## 驗證

- Docker C++17 colcon build 通過。
- Docker 獨立 recorder 測試以 `-Wall -Wextra -Werror` 編譯，
  驗證原始整數 timestamp、容量 2 下保留前兩筆並計數一次 omission、
  路徑不可寫時回報 I/O exception。
- 原速、viewer 關閉的真實 backend 回放結果見下方本輪紀錄。
- 未執行 viewer、硬體、多執行緒或正式 Stereo 回歸。

## 解讀限制

DDS message-lost callback 並非所有 KEEP_LAST 丟棄都會觸發；零事件不代表零遺失。
若一段 tracking 中沒有 IMU callback，只能證實 executor 無法在該段接收，
不能單憑此區分 player 發送、DDS cache overwrite、傳輸或系統排程原因。
CPU 百分比由 process 全部 threads 的 tick 差計算，100% 代表一個 CPU core，
多核可超過 100%。RSS 或平均 CPU 正常不排除短暫停頓。

本輪 artifacts 將保留在已忽略的 `final/diagnostics/`；不納入 commit。

## 本輪診斷回放結果（2026-09-23）

**本次完整原速回放通過，但沒有重現先前 DataGap，不能宣稱修復或根因已確定。**

| 項目 | 結果 |
|---|---|
| Source / replay | 原始完整 bag，1.0×，viewer 關閉，domain 95 |
| 左右原始接收 | 各 9,123，日誌逐筆核對與來源一致 |
| IMU | received=accepted=60,364，原始整數 timestamp 全數對應來源，無遺失／拒絕／overflow |
| Tracking | 9,122，130 NotInitialized、8,992 Ok；保留最後一對同步器缺口 |
| 初始化／reset | VIBA 1/2 出現；未見 reset，後續狀態檢查通過 |
| Pending | 最終 0、峰值 12，曾積壓但後來恢復 |
| Tracking 耗時 | mean 19.825 ms、P95 31.829 ms、max 54.961 ms |
| Trace | 166,340 events，dropped_events=0；7 項完整性核對通過 |
| IMU 相鄰 callback 最大間隔 | 89.507 ms，該兩筆 sensor 間隔 5.054 ms，沒有來源樣本遺失 |
| Node／player 退出 | 皆 0，無 cleanup signal；player 結束後約 5.278 秒 node 退出 |
| 系統監測 | 307 筆；node 最大約 378% CPU（約 3.78 cores）、player 約 20% |
| Node RSS | 最大 1,403,664 KiB，約 1.34 GiB |

CPU 為一秒區間的觀測最大值，不代表整台主機飽和；原始 host `/proc` 與
容器程序視角並不等同於完整 cgroup 資源限制調查。

沒有 IMU gap／DDS lost／拒絕／overflow 事件。回放完整性結果 `result.json`
10 項與 `analysis.json` 9 項皆通過，但不能把本輪結果改寫為前兩輪通過。
本輪最大 tracking 約 55 ms，不具備前次 2.13 秒長呼叫，也沒有 viewer 那輪持續增長到
26 pending 的情況，兩種失敗條件都沒有重現。

### 受控錯誤分支驗證

Docker 中以真實 ImuFrontend、ROS publisher／executor，依序送入時間
1.000、1.040、1.040、1.045 秒，gap 門檻 20 ms、buffer capacity 2：

- 4 received、3 accepted、1 duplicate rejection、1 overflow。
- 查詢 `(1.000, 1.040]` 回傳 DataGap，trace 保留 request 與相鄰 endpoints。
- 所有預期事件出現，dropped_events=0。測試來源與輸出保留於
  `final/diagnostics/frontend_check/`（不包含 build 產物）。
- 這是刻意生成的驗證資料，不是聲稱重現真實回放的失敗原因。

### 產物

[結果](final/diagnostics/result.json)、[原始 node 日誌](final/diagnostics/node.log)、
[trace CSV](final/diagnostics/trace.csv)、[trace 分析](final/diagnostics/trace_analysis.json)、
[系統原始採樣](final/diagnostics/resources.json)、[CPU/RSS 指標](final/diagnostics/resource_metrics.json)、
[版本紀錄](final/diagnostics/provenance.json)。

本步已完成診斷接線、受控錯誤分支與一次原速完整驗證。
若繼續根因調查，應保留診斷再次執行能觸發問題的情境，例如 viewer 原速對照；
是否新增接收執行緒仍須由實際 failure trace 決定。本步未自動執行第 3 步。
