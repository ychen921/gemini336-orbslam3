# T01 Stereo／Stereo-IMU baseline 比較報告

日期：2026-10-16。範圍：T01 tracking 統計與本輪 T03 基本時間、輸入及 queue 量測。

## 結論與驗收範圍

兩輪 `collection_complete=true`，frame 匯出、事件明細、最終 backend 計數及
程序收尾皆通過完整性檢查。Stereo 處理 9,111 frames，Stereo-IMU 處理
9,108 frames；兩輪 recorder overflow、overload discard 與 logger 遺失均為 0。

本輪 Stereo 出現一次 LOST entry，隨後建立新 map 並發生 merge；Stereo-IMU
沒有 LOST entry。兩模式有效處理速率均接近 30 Hz，adapter 平均時間分別為
17.693 ms 與 21.229 ms。這些是本次單輪結果，沒有定位 ground truth 或 pose
資料可用來判斷絕對精度、漂移或整體算法優劣。

**操作者已確認：「我並沒有改任何參數＆算法」。** 本報告採用這項確認，
將兩輪視為沿用各自既有模式設定的 baseline；Stereo-IMU 原有的 IMU 設定仍是
模式本身的差異。版本／hash 未隨 runtime 記錄，保留為來源限制，不要求為本次
報告重新 build 或重播。

兩份自動報告仍為 `comparison_ready=false`，原因是來源 metadata 未填；
此旗標不表示工具偵測到參數或算法改動。操作者確認與自動記錄的證據分開列示，
原始 metadata、分析報告與旗標均未回填或修改。

## 資料來源與實驗條件

| 項目 | Stereo | Stereo-IMU |
|---|---|---|
| Session | `20261016_032910_204565` | `20261016_051153_237213` |
| 自動分析結果 | [t01_report.json](../log_slam/t01_stereo/20261016_032910_204565/evaluation/t01_report.json) | [t01_report.json](../log_slam/t01_stereo_imu/20261016_051153_237213/evaluation/t01_report.json) |
| 逐 frame 資料 | [frames.csv](../log_slam/t01_stereo/20261016_032910_204565/evaluation/frames.csv) | [frames.csv](../log_slam/t01_stereo_imu/20261016_051153_237213/evaluation/frames.csv) |
| 事件明細 | [events.csv](../log_slam/t01_stereo/20261016_032910_204565/evaluation/events.csv) | [events.csv](../log_slam/t01_stereo_imu/20261016_051153_237213/evaluation/events.csv) |
| Console／STOP_PROCESS | [console.log](../log_slam/t01_stereo_console.log) | [console.log](../log_slam/t01_stereo_imu_console.log) |
| 設定檔路徑，依 metadata | `configs/Gemini_336_stereo.yaml` | `configs/Gemini_336_stereo_imu.yaml` |
| Evaluation／logging／viewer | 開啟／info／關閉 | 開啟／info／關閉 |
| Frame capacity／每筆大小 | 108,000／136 bytes | 108,000／136 bytes |
| ORB features／levels／scale，依 runtime log | 1,250／8／1.2 | 1,250／8／1.2 |
| FAST initial／minimum，依 runtime log | 20／7 | 20／7 |

實驗流程指定同一個 `bags/test_gemini336_stereo_imu/`，metadata 記錄
`evaluation.replay_rate=1.0`；此參數是操作設定紀錄，不是播放端速率量測。
兩輪記錄的 vocabulary 路徑相同。共同的左右影像 topics、2 ms 同步容差、
同步 queue size 10 亦保存在 `metadata.json` 的 `parameters_as_text`。

兩份 metadata 的 `bag_id`、`repository_commit`、`repository_dirty`、
`core_commit`、`core_dirty`、`settings_id`、`vocabulary_id` 皆為 `unknown`。
設定及 vocabulary 的路徑有紀錄，但沒有當時內容 hash 或 binary/source 對應證據。
上述 bag 路徑來自既有實驗流程，未將其寫回原始 metadata 當成 runtime 自動證據。

## 完整性與 tracking 結果

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| Processed／recorded frames | 9,111／9,111 | 9,108／9,108 |
| Raw state OK | 9,020 | 8,981 |
| 首次 OK 前的 NOT_INITIALIZED frames | 0 | 127 |
| NO_IMAGES_YET frames，含同次呼叫 reset | 1 | 0 |
| RECENTLY_LOST frames | 90 | 0 |
| Raw LOST frames | 0 | 0 |
| Source LOST entries | 1 | 0 |
| 成功 relocalization | 0 | 0 |
| 新 map／map switch／merge_applied | 1／1／1 | 0／0／0 |
| 有效 inlier samples／N/A samples | 9,019／92 | 8,980／128 |
| Inlier associations 平均／最小／最大 | 374.548／29／701 | 326.341／22／655 |
| 首次 OK 後的 ratio 分母 | 9,111 | 8,981 |
| RECENTLY_LOST ratio | 0.9878% | 0% |
| Raw LOST ratio | 0% | 0% |
| 最後一個 frame 的累計 KF | 777 | 1,270 |
| Backend join 後的最終累計 KF | 777 | 1,271 |
| Recorder dropped／overload discarded | 0／0 | 0／0 |
| Logger dropped／事件解析錯誤 | 0／0 | 0／0 |
| 事件明細筆數，含 map baseline | 5 | 1 |
| `collection_complete` | true | true |
| `comparison_ready` | false | false |

兩輪 `export_complete`、`frame_records_complete`、`event_details_complete`、
`event_counts_match`、`final_counters_consistent`、`process_ok` 皆為 true。
`accounting_valid=true`、最終 outstanding=0，與 console 的
`first_stop=InputIdle`、`exit_code=0`、`cleanup_failed=false` 一致。

Inliers 是 `TrackLocalMap` 的 persistent non-outlier feature associations，
不是 unique MapPoint 數量或精度分數。初始化及未執行該量測的分支為 N/A。
KF 是本輪新增 Atlas admissions 的累計，排除重複加入及 merge 搬移；
Stereo-IMU 在最後 frame 後仍新增一筆，因此最終值採用 join 後的 1,271。

### Stereo 事件順序

| 序號 | 事件 | 內容 |
|---|---|---|
| 1 | map_baseline | 啟動時已有 map 0，不算本輪新建 map |
| 2 | tracking_lost | Core frame 5779，使用 map 0 |
| 3 | map_created | 建立 map 1 |
| 4 | map_switched | map 1 切換到當時 ID 為 0 的 map |
| 5 | merge_applied | source map 1 搬入 target map 0，result ID 為 1；KF 576／4 |

事件依來源 `steady_ns`、`sequence` 排序，與最終 Core 計數一致。Frame 5779
在同一次呼叫內建立新 map，回傳 raw NO_IMAGES_YET，但 source LOST entry
仍有保留；因此 raw LOST ratio=0 不代表沒有 LOST。`merge_applied` 表示 topology
搬移已套用，不代表後續 BA 已全部完成；map ID 亦可能在 merge 時重寫。

Stereo-IMU 只有啟動 map baseline；其餘 T01 事件的最終計數均為 0，
與只有一筆事件明細相符。

## T03 基本時間與輸入／queue 結果

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| Adapter 平均，ms | 17.693 | 21.229 |
| Adapter P50／P95／P99，ms | 16.690／29.993／33.516 | 20.672／32.641／38.083 |
| Adapter 最大，ms | 127.303 | 138.667 |
| Enqueue-to-return 平均，ms | 21.859 | 29.180 |
| 首次 enqueue 至最後 return 的 throughput，Hz | 29.954 | 29.928 |
| 左／右 raw image received | 9,114／9,118 | 9,110／9,121 |
| Synchronized／converted | 9,111／9,111 | 9,108／9,108 |
| Outstanding admission peak／capacity | 8／30 | 10／30 |
| Image header 最大 interval，ms | 100.121 | 100.121 |
| Accepted IMU 最大 interval，ms | N/A | 70.611 |
| IMU received／accepted | N/A | 60,351／60,351 |
| IMU duplicate／backwards／overflow | N/A | 0／0／0 |
| 最終 buffered IMU samples | N/A | 13 |

Adapter 時間涵蓋輸入驗證、必要的 IMU 轉換、`TrackStereo()` 與 cached state
讀取；recorder 更新在計時區間外。Percentiles 採 nearest-rank，且本輪沒有
recorder overflow，因此涵蓋全部正常返回的呼叫。Throughput 使用同程序的
steady clock，排除首次入隊前及最後返回後的等待；不是整個 node 存活期間平均。

本輪兩個模式沒有 overload discard，queue admission peak 低於 capacity。
這不證明 bag 的每一筆 message 都已被 ROS 接收；raw reception、同步與已入隊
工作是不同範圍。Image／IMU gaps 的來源尚未判定，接收路徑根因維持暫緩。

## 共同 sensor timestamp 的補充對照

兩輪最早及最晚的 sensor timestamp 相同，分別為
`1789957367563022000` 與 `1789957671858324000` ns，跨度 304.295302 秒。
Stereo 在第一個 frame 已為 OK；Stereo-IMU 首次 OK 的 sensor 時間距首 frame
為 4.238431 秒。這是首次 raw OK 的時間，不能單獨代表完整慣性初始化完成。

以整數 `sensor_timestamp_ns` 精確相等取得交集，共有 9,100 筆；Stereo 另有
11 筆、Stereo-IMU 另有 8 筆不在交集。兩輪各自沒有重複 sensor timestamp。
從交集中再保留雙方 `tracked_mappoint_inliers_valid=1` 的 8,884 筆，得到：

| 共同有效樣本指標，n=8,884 | Stereo | Stereo-IMU |
|---|---:|---:|
| Inlier associations 平均 | 371.033 | 327.631 |
| Adapter 平均，ms | 17.825 | 21.489 |
| Adapter P50／P95／P99，ms | 16.840／30.052／33.580 | 20.897／32.718／38.117 |

此表重新由 `frames.csv` 計算，percentile index 為 `ceil(n*p)-1`。
它讓當前 frame 的 sensor timestamp 與有效量測條件一致；兩模式先前接收歷史、
map／KF、loop／BA 及執行緒排程仍可能不同，不能把這項配對視為所有條件受控。

## 限制與後續

- 只有每模式一輪、同一個 bag 的描述性比較；不據此宣告一般化的效能或精度優劣。
- 操作者確認參數及算法未調整；當時 commit、dirty 狀態與內容 hash 沒有自動記錄。
  因此本報告可供 baseline 審閱，但沒有補成完整來源可重現性證據。
- Inliers 與 KF 數量受各模式追蹤及建圖流程影響；較大數值不直接代表較高精度。
- Stereo-IMU 日誌的四筆 `NOT INERTIAL LINK` 均伴隨 `KF=0`、
  `IS_ORIGIN=true`、`REASON=NO_PREV_KF`、`EDGE_ADDED=false`，對應 origin
  沒有 predecessor 的跳過分支；這些訊息未提供非 origin KF 缺鏈的證據。
- 本輪皆正常結束；先前間歇性 Stereo Sophus／NaN abort 的根因仍未解決。
- 沒有 evaluation OFF 對照，不能從兩模式時間差推論 recorder 的採集開銷。
- T02 pose、map/version 與閉環 endpoint 差異尚未量測，也沒有 ground-truth ATE/RPE。

目前待操作者審閱本報告。若需量化採集開銷，再規劃同模式 ON／OFF 多輪對照；
T02 則先規劃最小 pose 與 map/version 介面，經審閱後才實作及人工採集。
詳見 [validation plan](../docs/validation_plan.md)。本次僅離線讀取既有資料並撰寫文件，
未執行 build、單元測試、rosbag replay 或 SLAM runtime。
