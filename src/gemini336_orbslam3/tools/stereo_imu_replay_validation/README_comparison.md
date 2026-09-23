# 6D-2：既有兩輪日誌比較（分析第 1 步）

本輪只實作離線分析工具，沒有修改正式 C++、重跑 bag 或處理異常 keyframe pose。
輸入為 `final/full_bag/node.log`、`docs/6d2-viewer-node.log` 及同一份來源時間戳清單。
來源 SHA-256 記錄於輸出的 `comparison.json`，分析產物放在已忽略的 `final/comparison/`。

## 結果

| 項目 | 無 viewer 驗證 | 手動 viewer |
|---|---:|---:|
| 完成 tracking | 4,688 | 7,441 |
| 原始左右接收 | 各 4,695 | 各 7,469 |
| 最後接收到的來源影像索引（從 1 起算） | 4,794 | 7,471 |
| 最後接收時間之前缺少的左右影像 | 各 99 | 各 2 |
| 最終 pending | 5 | 26 |
| 報錯取樣區間相對 bag 第一張影像 | (156.689522, 159.792221] 秒 | (248.401739, 248.435111] 秒 |
| 原始 IMU 在報錯區間的最大間隔 | 5.057 ms | 5.050 ms |
| 原始 IMU 樣本數（含左右參考） | 617 | 8 |

兩輪均有來源 IMU 覆蓋；實際 runtime 缺失樣本仍未記錄，不能由 image reception 推算。
未接收的後半段資料不列入上述缺失計數。

### 無 viewer：短時停頓與接收缺口

- 來源第 3～4 張缺少，不與已記錄的 100 ms 以上 tracking 呼叫重疊。
- 約 108.33～108.46 秒缺第 3,247～3,251 張，callback 間隔約 337 ms，
  與第 3,243 次 tracking 的約 334 ms 呼叫時間估計重疊。
- 第 4,605 張及第 4,697～4,712 張缺口已出現在最後 2.13 秒呼叫之前。
- 第 4,714～4,788 張（左側）缺口與第 4,688 次長 tracking 呼叫估計重疊。
  右側接收位置略異，詳見 comparison.json；不能把所有缺口歸因於同一次呼叫。
- Pending 多數快照為 0，無法據此排除上游接收問題。

### Viewer：持續處理較慢與積壓

第二次 loop 在約 232.25 秒附近。以影像 header 分段統計：

| Bag 區間 | Tracking 平均耗時 |
|---|---:|
| [225, 232) 秒 | 15.002 ms |
| [232, 237) 秒 | 31.280 ms |
| [237, 242) 秒 | 34.403 ms |
| [242, 最後完成幀約 248.402] 秒 | 35.584 ms |

後兩段平均呼叫耗時已超過約 33.4 ms 的影像週期，尚不含其他 callbacks。
Pending 由 1、10、18 增至 26，最舊等待 0.946 秒；這是持續跟不上輸入的直接證據。
左右接收僅缺來源第 7,397、7,399 張（約 246.833、246.900 秒），仍報 IMU DataGap。
所以「影像缺少很多」不是 runtime IMU 缺口的必要條件。

第二次 loop 與耗時上升有時間關聯，但兩輪並非受控 viewer A/B 實驗，
目前不能判定 loop、viewer 或 IMU 模型造成了耗時變化。

## 時間軸與限制

[時間線 PNG](final/comparison/timeline.png)／[SVG](final/comparison/timeline.svg)。
上排為每一 sensor 秒的 tracking 平均／最大耗時；中排為定期 pending 快照；
下排為以第一個左影像 callback 對齊的相對返回延遲。
綠色虛線標 loop 附近位置，紅色區段標左影像缺口；左右圖座標範圍不同。

- 以整數奈秒與 Decimal 解析，先相減再換秒，避免直接相減 epoch 浮點數。
- Loop 原始訊息無 timestamp，使用前後 tracking 日誌的 header 時間形成 bracket。
  非同步輸出有緩衝，不能把 bracket 當精確事件時間或因果證據。
- Tracking 耗時用 steady clock 量測，但既有 ROS log timestamp 是另一時鐘。
  用返回日誌時間減 tracking 耗時只是近似呼叫區間，還含返回後統計／日誌延遲。
- 相對延遲會出現負值，因首個 callback 本身可能較晚；不是負的物理延遲，
  也不是 sensor-to-result latency。圖中不以線性漂移推斷排程根因。
- Pending 只有定期快照；橫軸用該行之前最後完成的影像時間，重複 x 值可能代表
  tracking 已停止但清理尚未完成。不能據圖認定峰值或精確持續時間。
- 無 runtime IMU 逐筆紀錄、DDS 遺失統計與系統負載，不宣稱已確定丟資料的層級。

## 重跑分析

只需 Python 標準函式庫；輸出目錄必須不存在，以免覆寫證據。

```bash
python3 src/gemini336_orbslam3/tools/stereo_imu_replay_validation/compare_logs.py \
  --baseline src/gemini336_orbslam3/tools/stereo_imu_replay_validation/final/full_bag/node.log \
  --viewer docs/6d2-viewer-node.log \
  --source-timestamps src/gemini336_orbslam3/tools/stereo_imu_replay_validation/final/full_bag/clip_timestamps.json \
  --output /tmp/gemini336-log-comparison-new
```

輸出 comparison.json 與兩輪逐秒 CSV。本輪圖表另使用主機已安裝的 matplotlib 3.10.8，
繪圖腳本存於本機 artifact `final/comparison/plot.py`；未新增專案依賴。
圖表、輸入日誌與 JSON 都不納入 commit。

## 第 2 步建議審閱範圍（尚未實作）

優先補 IMU 相鄰 accepted samples 的實際 gap endpoints，以及 callback、tracking
開始／結束的同一 steady-clock 時間線，並同步觀察 node／player 系統負載。
特別要區分：callback 從未收到、validator 拒絕、buffer 淘汰、取樣錯誤四個位置。
採有界記憶體紀錄並輸出滿載計數，避免新增大量同步磁碟輸出干擾結果。
暫不調大 queue、不放寬 max_gap、不拆執行緒，先量測原因。
