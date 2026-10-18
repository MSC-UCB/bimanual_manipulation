# RealSense 相機整合：最小改動方案

日期：2026-10-14。此文件是實作提案，尚未修改 launch 或 viewer。

## 需求與命名

在現有 launch 新增兩個布林參數，預設皆為 `false`：

| ROS launch 參數 | 啟用裝置 | 預設 |
| --- | --- | --- |
| `enable_wrist_camera` | 兩台 D405，分別作為左／右手腕相機 | `false` |
| `enable_fix_camera` | 一台固定視角相機，依需求先以 D415 規劃 | `false` |

這裡把「prefix」理解為新增啟用參數；實際命令使用 `enable_wrist_camera:=true`，不是 `-enable wrist camera`。既有 `prefix:=left_ / right_` 繼續用於選擇手臂／手，不與相機開關混用。

固定相機先前提到可能是 D435，後續需求指定 D415；待接上後確認型號與序號。兩者都採用 RealSense ROS 驅動，開關名稱不需隨型號更改。

## 已確認的專案結構

- `launch/bimanual_system.launch.py`：雙臂、雙手與感測器 viewer；`viewer` 預設 `true`。
- `launch/single_arm_hand.launch.py`：單臂與同側手；`viewer` 預設 `false`。
- `bimanual_manipulation/tactile_wrench_viewer.py`：OpenCV 視窗，目前為 1400 × 870，包含 wrench、觸覺熱圖與 F6；由 ROS subscription 接收資料，已有 `NO DATA`、`STALE`、`INVALID DATA` 及 snapshot 功能。
- `setup.py` 已用 glob 安裝 `launch/*.launch.py` 和 `config/*.yaml`，新增這兩類檔案不需另外登錄。
- `package.xml` 已有 `sensor_msgs`、NumPy、OpenCV；需補 `realsense2_camera` 與 `cv_bridge` 執行依賴。
- 本機標準 Jazzy 安裝路徑中有 `cv_bridge`，未看到 `realsense2_camera`；實作前確認已 source 的 overlay 是否提供驅動，若無則安裝相容版本。

目前偵測到的 D405 序號為 `260522273536` 與 `260522275388`。尚未確認哪台裝在左腕、哪台在右腕，不能以 USB 掃描順序決定。

## 最小架構

沿用官方 `realsense2_camera/rs_launch.py`，不自行撰寫 RealSense SDK 擷取節點。每台相機各一個獨立命名的驅動實例，viewer 只訂閱影像。

```text
bimanual_system.launch.py / single_arm_hand.launch.py
  ├─ 既有 arm、hand、description
  ├─ cameras.launch.py（依兩個開關啟動 0／1／2／3 台）
  │   ├─ wrist_left  D405
  │   ├─ wrist_right D405
  │   └─ fixed       D415，實機型號待確認
  └─ tactile_wrench_viewer.py（既有感測器 + 三個影像格）
```

### Launch 與設定

1. 新增 `launch/cameras.launch.py`，集中處理兩個開關、序號設定與官方 launch 的 include。兩個既有 system launch 都轉交相同參數，避免複製三份驅動設定。
2. 新增 `config/cameras.yaml`，以 `wrist_left`、`wrist_right`、`fixed` 三個角色保存序號與串流設定，由相機 launch 讀取。序號使用字串；D405 左右位置需人工核對後填入，固定相機序號先留空。這是本套件的角色設定檔，不直接當成官方 driver 的 ROS parameter YAML。
3. 僅對已啟用的角色解析 RealSense package 路徑、讀取／驗證所需設定並啟動 driver。兩個開關皆為 `false` 時，不應因為未安裝相機驅動或沒有序號而使原有 launch 失敗。
4. 啟用角色的序號不可留空或重複；不使用「第一台可用相機」作為 fallback。設定有誤時回報角色與原因，避免左右互換或固定相機誤抓 D405。官方 launch 的 `serial_no` 字串格式依實際安裝版本確認，必要時使用它要求的底線前綴。
5. 相機各自使用隔離的 launch scope，例如 `GroupAction`，避免官方 include 的同名參數互相影響。建議設定 `camera_namespace:=/camera`，`camera_name:=wrist_left / wrist_right / fixed`，並確認實際 topic 與 frame 名稱各自唯一。
6. `enable_wrist_camera:=true` 在兩個入口都代表啟用兩台 D405；單臂入口的 `prefix` 不改變這個定義。`viewer:=false` 時相機仍可發布影像；單臂入口要顯示畫面需另外給 `viewer:=true`。
7. 相機開關獨立於 `crx_mock`、`sharpa_backend` 與 `read_only`，因此可以用模擬手臂／手驗證真實相機畫面。

預期 viewer 訂閱以下 `sensor_msgs/Image` topics；實作時以安裝版本實際發布名稱核對：

| 角色 | 預期彩色影像 topic |
| --- | --- |
| 左腕 D405 | `/camera/wrist_left/color/image_raw` |
| 右腕 D405 | `/camera/wrist_right/color/image_raw` |
| 固定相機 | `/camera/fixed/color/image_raw` |

第一版以每台一個彩色畫面為目標。先嘗試裝置支援的 640 × 480、30 FPS profile，最後以 SDK／driver 列出的模式為準。關閉 point cloud、depth alignment 與額外濾波；depth 是否可關閉需特別驗證 D405 的彩色串流需求，不直接假定三台的參數完全相同。若 USB 頻寬不足，再降到裝置支援的較低幀率。

### Visualizer 的三個畫面

直接擴充 `tactile_wrench_viewer.py`，不建立新 GUI 或額外相機視窗。這裡的 visualizer 指現有感測器 viewer；RViz 繼續顯示機器人模型。

- 兩個相機開關皆關閉：保留原本 1400 × 870 版面，不新增相機訂閱。
- 任一開關開啟：右側新增約 480 px 寬的相機欄，縱向放三格：`LEFT WRIST`、`RIGHT WRIST`、`FIXED`。保持影像比例，空白處補邊；視窗可縮放。
- 關閉的角色顯示 `DISABLED`；啟用但未收到資料顯示 `NO DATA`；停更超過既有 `stale_after` 門檻顯示 `STALE`；轉換失敗顯示 `INVALID DATA`，不讓整個 viewer 退出。
- 新增 viewer CLI flags `--enable-wrist-camera`、`--enable-fix-camera`，以 `store_true` 實作，預設皆關閉。Launch 只在對應布林為真時附加 flag；不要傳入字串 `false` 給 `store_true`。
- 用 `cv_bridge` 轉成 OpenCV 的 BGR 影像，避免自己處理 encoding、stride 和 RGB/BGR 差異；只在相機功能啟用時載入。每台只保留最新一張，沿用 `BEST_EFFORT`、queue depth 1，避免積壓影像。
- 沿用目前 `spin_once`／render loop；相機收不到資料不能阻塞力覺與觸覺畫面。中斷相機後標示過期，不能繼續把最後一張當成即時畫面。
- Snapshot PNG 同時包含三個影像格，JSON 增加各相機狀態；相機功能關閉時保留原本輸出行為。

第一版不新增相機外參、URDF 安裝座、點雲、錄影或跨相機同步。僅顯示 2D 影像不需要先完成手眼標定；未標定的相機 frame 不假裝與 `world` 已對齊。

## 預計修改檔案

| 檔案 | 修改內容 |
| --- | --- |
| `launch/bimanual_system.launch.py` | 新增兩個預設 false 的參數，include 相機 launch，轉交 viewer flags |
| `launch/single_arm_hand.launch.py` | 同樣加入參數、include 與 viewer flags |
| `launch/cameras.launch.py`（新增） | 依角色與序號啟動官方 RealSense driver |
| `config/cameras.yaml`（新增） | 三台相機的角色、序號與串流設定 |
| `bimanual_manipulation/tactile_wrench_viewer.py` | 相機訂閱、最新影像快取、三格顯示與狀態 |
| `package.xml` | 宣告 `realsense2_camera`、`cv_bridge` 執行依賴 |
| `README.md` | 記錄啟用方式、序號對應與相機安裝需求 |

## 實作後的預期用法

以下參數目前尚未實作；完成後即可加到原本的 launch 命令。

```bash
# 維持現況：相機預設全部關閉
ros2 launch bimanual_manipulation bimanual_system.launch.py

# 目前先驗證兩台 D405，固定相機格顯示 DISABLED
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  enable_wrist_camera:=true viewer:=true

# 三個畫面都開啟（固定相機接好並填入序號後）
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  enable_wrist_camera:=true enable_fix_camera:=true viewer:=true

# 單臂入口也可開兩台手腕相機；viewer 必須明確開啟
ros2 launch bimanual_manipulation single_arm_hand.launch.py \
  prefix:=left_ enable_wrist_camera:=true viewer:=true
```

## 最小驗證範圍

1. 檢查兩個入口 `--show-args`：新參數預設均為 `false`。全部關閉時，原有 launch／viewer 行為正常，且不解析 RealSense driver 路徑。
2. 用 mock 手臂／手啟動兩台真實 D405：確認兩個 topic、序號與畫面對應，力覺／觸覺更新不受影響。先遮住一台鏡頭，核對左右標籤。
3. 對缺失／重複序號、RGB/BGR 轉換、影像停更狀態做小範圍檢查；拔掉一台時另一台與感測器畫面仍可更新。
4. 核對 PNG／JSON snapshot 包含影像及狀態；`viewer:=false` 仍能單獨發布相機 topics。
5. 固定相機接上後，確認 D415／D435 型號、序號、profile 與三台同時串流。檢查固定相機單獨啟用及三台全開兩種組合；目前不把第三台列為已驗證。

本次只閱讀現有程式並撰寫提案，未安裝依賴、啟動機器人或進行相機串流測試。
