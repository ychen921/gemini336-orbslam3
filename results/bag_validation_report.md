# Stereo 與 Stereo IMU 分析報告

評估日期：2026-10-28。使用同一份錄製資料分別執行 Stereo 與 Stereo-IMU，比較最終軌跡首尾一致性、追蹤品質、處理效能及輸入品質。

**本次 Stereo-IMU 的起終點位置差較小、處理時間較短，且沒有 overload 丟幀；Stereo 的起終點角度差較小。兩種模式中途都建立新地圖並完成合併，因此不能將結果解讀為全程無中斷追蹤。**

本次只有一組 paired runs。Stereo 缺少對應 console log，首尾數字保留為診斷值；Stereo-IMU 通過資料採集完整性檢查。這是回原點一致性評估，不是有 ground truth 的全軌跡絕對精度評估。

## 測試條件

操作者先移動讓 Stereo-IMU 初始化，再回到原點停留數秒，之後開始正式路徑並回到起點。操作者估計約第 13 秒回到原點；軌跡在 13–15 秒仍有明顯位移，因此共同起始窗口採 15–18 秒。這個窗口位於操作者所述的返回停留階段，不是直接替換為第一筆有效 pose。

| 條件 | Stereo | Stereo-IMU |
|---|---|---|
| 記錄的 replay rate | 1.0 | 1.0 |
| 視覺化介面 | 關閉 | 關閉 |
| 評估資料與最終軌跡記錄 | 啟用 | 啟用 |
| 第一筆 OK 狀態 | +0.033369 s | +3.704105 s |
| 第一筆 IMU initialized context | 不適用 | +6.340359 s |

時間皆相對兩輪已記錄感測資料的共同起點，記錄跨度為 1060.924341 秒，尚未與原始錄製資料的起點核對。兩輪採相同同步容許差 2 ms、影像處理容量 30 及 5 秒影像逾時關閉設定；相機設定分別對應 Stereo 與 Stereo-IMU 模式。軟體版本及設定內容缺少唯一識別資訊，無法完整驗證執行條件；紅外線投射器狀態未記錄。

## 起終點一致性

起始窗口為 +15–18 秒，結束窗口為 +1057.924341–1060.924341 秒。兩種模式皆使用各自最終地圖重建後的左相機 pose，位置採等權平均，旋轉採 quaternion 正負號不敏感的 SO(3) chordal mean。

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| 起終點平移差 | **0.965204 m**，診斷值 | **0.054633 m** |
| 起終點旋轉差 | **2.645505°**，診斷值 | **4.001860°** |
| 起始／結束窗口樣本數 | 90／90 | 90／89 |
| 起始窗口平移 RMS | 1.574 mm | 10.862 mm |
| 結束窗口平移 RMS | 1.658 mm | 2.433 mm |
| 起始窗口旋轉 RMS | 0.040556° | 0.125154° |
| 結束窗口旋轉 RMS | 0.022866° | 0.020989° |

Stereo-IMU 的平移差約為 Stereo 的 1/17.7；角度差則較大。RMS 表示窗口內估計 pose 相對平均 pose 的變動，不等於實際擺放精度或獨立的靜止證明。首尾差包含重放估計、實際位置與方向的擺放差異，不能拆解成純 SLAM drift。

Stereo 最終軌跡匯出完整，但缺少程序結束與記錄遺失情況的確認資料。因此其首尾數字僅作診斷比較，尚未通過完整性確認。

## 追蹤品質與地圖事件

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| 已處理／已記錄幀數 | 31,477／31,477 | 31,624／31,624 |
| OK 狀態幀數 | 31,386 | 31,398 |
| NOT_INITIALIZED 幀數 | 0 | 225 |
| NO_IMAGES_YET 幀數 | 1 | 1 |
| RECENTLY_LOST 幀數 | 90 | 0 |
| LOST 狀態幀數 | 0 | 0 |
| Tracking lost 來源事件 | 1 | 0 |
| Relocalization 來源事件 | 0 | 0 |
| 新建 map／merge applied | 1／1 | 1／1 |
| Active map reset | 0 | 2 |
| 累計新增 KeyFrame，shutdown 後 | 1,611 | 3,588 |
| 有效 inlier 樣本數 | 31,385 | 31,392 |
| Inlier 平均／最小／最大 | 348.95／17／669 | 321.62／36／649 |

Inlier 是 TrackLocalMap 的 persistent non-outlier feature associations，不是唯一 MapPoint ID 數。累計新增 KeyFrame 保留重置前的新增量，排除 merge transfers，不等於最終地圖的 KeyFrame 數。

Stereo 在第一次 OK 後的 31,477 幀中，RECENTLY_LOST 比例為 0.2859%。Stereo-IMU 的 LOST 與 RECENTLY_LOST 都是 0，但其 timestamp-jump 路徑會返回 NO_IMAGES_YET，再經 NOT_INITIALIZED 恢復，因此這些零值不能證明追蹤從未中斷。

## 白牆區段與恢復行為

### 觀察

操作者確認該路段遇到白牆，缺少 features，且沒有 stereo 配對偏差。兩輪已處理影像都在 +339.784570 至 +344.855188 秒之間出現 **5.070618 秒**的時間戳間隔。

| 後續事件 | Stereo | Stereo-IMU |
|---|---|---|
| 斷點後狀態 | RECENTLY_LOST，90 幀 | Timestamp jump warning，轉 NO_IMAGES_YET |
| 建立新 map | 約 +347.859 s | 約 +344.855 s |
| 額外重置 | 無 | 約 +349.2、+353.1 s 各一次 |
| 重置相關 log | 不適用 | `Not enough motion for initializing`、bad IMU flag |
| Merge applied | 約 +380.7 s | 約 +379.9 s |

沒有 sensor timestamp 的非同步事件，其相對時間以最接近事件 steady time 的已處理影像估計，只供定位，不是事件的精確 sensor time。

### 假設

白牆缺少特徵支持 tracking 失敗與重建地圖的解釋。依操作者現場觀察，配對偏差不列為主要假設。惟單純特徵不足不必然造成 wrapper 的已處理影像缺口；現有單側影像接收 counters 也沒有 5 秒最大間隔，因此這個已處理影像缺口的形成原因仍未確認。

### 結論

兩模式都在此區段中斷原本的追蹤／地圖流程，之後以新 map 再 merge。Stereo-IMU 多了兩次重新初始化相關 reset，不能宣稱其白牆恢復能力較好。較小的最終位置差與較低的處理成本，是這次資料直接支持的優勢。

## 處理效能

處理時間採整合層量測的 SLAM 呼叫期間，包含呼叫內部等待，不等同於純最佳化運算時間。以下全輪百分位數使用採集時輸出的統計值。

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| SLAM 呼叫平均時間 | 27.83 ms | 21.19 ms |
| P50／P95／P99 | 27.49／37.24／41.59 ms | 20.16／30.70／37.43 ms |
| 最大值 | 290.94 ms | 271.58 ms |
| Enqueue 到 return 平均 | 123.53 ms | 24.46 ms |
| Pending 平均 | 2.644 | 0.013 |
| Outstanding 峰值／容量 | 30／30 | 9／30 |
| Overload discarded | 41，佔 enqueue 的 0.1301% | 0 |
| Host throughput | 29.66 Hz | 29.81 Hz |

Host throughput 的範圍為第一筆 enqueue 至最後一筆 return，不包含節點啟動與全部 shutdown 等待時間。Stereo 的 P95 超過 30 Hz 的 33.3 ms 週期，且有 queue 滿載與明確 overload discard；丟幀結論來自 counters，不只來自 P95。

另取 +15 秒後、相同 sensor timestamp、兩輪皆 OK 且 inlier count 有效的 **30,858 幀**，排除不同初始化及非 OK 樣本的直接影響：

| 共同有效樣本指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| SLAM 呼叫平均時間 | 27.97 ms | 21.29 ms |
| P95／P99 | 37.30／41.65 ms | 30.71／37.53 ms |
| Inlier 平均 | 347.65 | 320.70 |

此範圍內 Stereo-IMU 的平均 SLAM 呼叫時間 約少 24%。共同樣本百分位數採 `(N−1)×p` 線性插值；兩輪仍是分開執行，不代表相同 host 負載或相同內部地圖。

## 影像與 IMU 輸入品質

| 指標 | Stereo | Stereo-IMU |
|---|---:|---:|
| 左／右影像接收數 | 31,681／31,767 | 31,780／31,787 |
| 左／右影像間隔平均 | 33.489／33.400 ms | 33.385／33.379 ms |
| 左／右影像間隔最大 | 133.484／100.111 ms | 133.481／133.482 ms |
| 左／右影像 gap 超過 100 ms 次數 | 4／2 | 1／1 |
| 影像重複／倒退 | 皆 0 | 皆 0 |
| 已同步並轉換影像對 | 31,518 | 31,624 |
| 空影像／大小不符／轉換失敗 | 皆 0 | 皆 0 |
| Evaluation recorder dropped | 0 | 0 |
| Input accounting valid | true | true |

Stereo-IMU 接收並接受 210,362 筆 IMU，間隔平均 5.043821 ms、最小 4.919052 ms、最大 5.171776 ms。沒有超過設定 20 ms 的 accepted IMU gap，沒有 duplicate、backwards、overflow 或 invalid value／timestamp。Buffer 峰值為 1,016，容量為 2,000。

這些是接收端與已接受樣本的統計，不保證 bag 全量訊息均已接收。Recorder dropped、wrapper overload discarded、同步前未配成對的影像，以及最終軌跡排除列屬於不同範圍，不能合併成同一個丟幀率。

## 資料完整性與評估限制

兩輪逐幀資料匯出完整，解析到的事件數與結束後的累計計數一致。Stereo-IMU 已確認正常退出，且記錄訊息無遺失；Stereo 缺少對應的程序結束紀錄，因此完整性仍未確認。兩輪也都有軟體版本與設定識別資訊不足的限制。

本輪完成首尾診斷、追蹤品質、處理時間、輸入與佇列分析。單次比較尚不能取代重複性測試，也未量化評估資料記錄本身的開銷。雖觀察到閉環修正流程，但缺少同一關鍵幀修正前後的配對資料，因此尚未量化閉環修正量。

### 後續評估

若先以此組資料作未來 report 的基線，保留完整性與低紋理區段的限制即可。若要釐清 5.070618 秒已處理影像缺口，可另行對齊該區段的輸入、同步與 tracking 記錄；這與白牆造成的特徵不足分開判定。後續多輪手動 replay 應保留兩種模式的 console、bag 識別與 source／settings／vocabulary 資訊，再評估結果重複性。

本報告只使用既有 runtime 產物與操作者觀察，未重新執行 ROS、bag replay 或硬體驗證。
