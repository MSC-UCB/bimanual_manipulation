# bimanual_manipulation：程式調查與最小整合計畫

> **本地實作更新（2026-09-24）：** 已建立 ROS package；目前的編譯、mock 預覽與 hold 操作請看 [README.md](README.md)。本地 `dual_crx_control` checkout 為 `5f30342`，已移除本計畫下文使用的 `dual_arm_teleop.launch.py`、`/teleop/*` 與 `teleop_bridge`；實作改用 `dual_arm.launch.py`、`/crx5ia/joint_targets`（`JointState`）及左右獨立回饋。合併 URDF/RViz 已完成，安裝位姿仍為 mock 暫定值；可選的小幅動作尚未實作。以下保留原始調查記錄，舊介面範例不可直接照做。

調查日期：2026-09-23；整機 description 與總 launch 設計更新：2026-09-24（美西時間）。目標是**先以最少改動接通雙 CRX 與雙 Sharpa Wave**；本文件是交給 Codex 實作的規格，尚未建立或執行新的 ROS package。以下依兩個 repository 的 `main` 分支檢查，CRX tree `34a8ca3`、Sharpa tree `4bc82a6`；實機啟動前應比對本機 checkout。

## 1. 已確認的現況

| 系統 | GitHub repository | ROS package 名稱 | 現成啟動方式 |
| --- | --- | --- | --- |
| 左右 CRX | [MSC-UCB/dual_crx_control](https://github.com/MSC-UCB/dual_crx_control) | `dual_crx_control` | `dual_arm_teleop.launch.py`，`mock:=true/false` |
| 左右 Wave | [MSC-UCB/dual_sharpa_wave_ros2](https://github.com/MSC-UCB/dual_sharpa_wave_ros2) | `dual_sharpa_wave`（**不是** repo 名稱） | `dual_sharpa.launch.py`，`backend:=mock/sharpa_sdk` |

兩邊文件和 launch 都以 ROS 2 Jazzy 為目標；本機執行前仍需用 `echo $ROS_DISTRO` 核對。CRX teleop launch 已啟動兩臂 driver/controller、`interpolation_node`、`teleop_bridge`；Sharpa launch 已啟動左右兩個獨立的 `hand_node`。新專案新增 ROS 2 Python package `bimanual_manipulation`，沿用兩邊的控制 topic，另外建立一份整機 description 和一個總 launch。不改兩個原有 package。總 launch 只負責啟動既有控制系統與整機 TF；測試命令由獨立 node 明確啟動。

### 確認過的介面

| 方向 | Topic | ROS message | 格式與注意事項 |
| --- | --- | --- | --- |
| 送給兩臂 | `/teleop/joint_command` | `std_msgs/msg/Float64MultiArray` | 恰好 12 個有限數值，弧度；前 6 個左臂 J1–J6，後 6 個右臂 J1–J6。保持發布，初版以 100 Hz 對應 launch 預設 `input_rate_hz:=100.0`。 |
| 讀取兩臂 | `/teleop/joint_states` | `sensor_msgs/msg/JointState` | 左右各 6 個，依 `name` 對應 `left_J1`…`left_J6`、`right_J1`…`right_J6`，不可假設陣列順序。另有 `/left/joint_states`、`/right/joint_states`。 |
| 送給左手 | `/sharpa/left_hand/joint_command` | `sensor_msgs/msg/JointState` | 左手完整 22 個位置，弧度；提供完整 canonical `name` 與 `position`。 |
| 讀取左手 | `/sharpa/left_hand/joint_states` | `sensor_msgs/msg/JointState` | 22 個關節實際 feedback。 |
| 送給右手 | `/sharpa/right_hand/joint_command` | `sensor_msgs/msg/JointState` | 右手完整 22 個位置，弧度；提供完整 canonical `name` 與 `position`。 |
| 讀取右手 | `/sharpa/right_hand/joint_states` | `sensor_msgs/msg/JointState` | 22 個關節實際 feedback。 |

Sharpa 的正式 namespace 是 `sharpa/{side}_hand`，因此**不要**用先前概念稿的 `/sharpa/left/command`。手的 canonical 關節名稱由 [`joint_names.py`](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/joint_names.py) 定義，例如 `left_thumb_CMC_FE`；`hand_node` 接受包含 22 個正確名稱的訊息並重新排序，或接受空 `name`、以 canonical 順序傳入 22 個位置。初版推薦填上完整 `name` 方便驗證。Sharpa 命令和 feedback 的 QoS 是 reliable、volatile、depth 1；新 node 的手部 publisher/subscriber 需相容。CRX bridge 訂閱外部命令時用 depth 1，讀取左右 driver feedback 時使用 sensor-data QoS。以上直接由 [`teleop/bridge.py`](https://github.com/MSC-UCB/dual_crx_control/blob/main/src/dual_crx_control/teleop/bridge.py)、[`hand_node.py`](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/hand_node.py) 及 [`qos.py`](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/qos.py) 確認。

CRX 的內部路徑是 `/teleop/joint_command` → `teleop_bridge` → `/interpolation/joint_targets` → `interpolation_node` → 兩臂 controller。整合 node 應使用上層 `/teleop/joint_command`，不要同時直接寫 controller topic。`interpolation_node` 在收到一次目標後會持續發布最後目標；停止上層 publisher 並非安全停止機制。[CRX 程式](https://github.com/MSC-UCB/dual_crx_control/blob/main/src/dual_crx_control/interpolation/node.py)

Sharpa SDK 端讀回實際關節位置；硬體設定中的 `command_timeout_sec: 0.0` 代表不啟用命令 timeout。`stop()` 尚未驗證為實體急停，斷掉 publisher 也不等於急停。[硬體設定](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/config/dual_sharpa_hardware.yaml)、[README](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/README.md)

## 2. 最小架構

```text
~/ws_fanuc/src/
├── dual_crx_control/               # 現有 CRX package
├── dual_sharpa_wave_ros2/           # 其中的 ROS package 名稱是 dual_sharpa_wave
└── bimanual_manipulation/          # 新建 ament_python package
    ├── bimanual_manipulation/
    │   ├── __init__.py
    │   ├── hold_and_test.py         # 實際命令來源，手動啟動
    │   └── composite_description.py # 合併既有 CRX xacro 與左右 Sharpa URDF
    ├── launch/
    │   ├── bimanual_system.launch.py      # include 原有兩個 launch + description
    │   └── bimanual_description.launch.py # 合併 joint states + 全域整機 TF
    ├── config/
    │   └── hand_mounts.yaml         # 左右安裝座 xyz/rpy；實機值待量測
    ├── resource/bimanual_manipulation
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

如果兩個舊 package 已安裝在不同 workspace，可用有明確順序的 overlay；初版以同一個 `~/ws_fanuc/src` 最直觀。先確認本機是否已經有 `dual_sharpa_wave`，避免重複 clone / overlay 到舊版本。維持單一 ROS_DOMAIN_ID 與相同 ROS 環境。不要另外起 `dual_arm.launch.py`，因為它和 teleop launch 會重複啟動同一對 CRX controller。

## 3. 給 Codex 的實作步驟（以最少 effort 為準）

1. **盤點本機環境。** 先檢查 `~/ws_fanuc/src`、`ros2 pkg prefix dual_crx_control`、`ros2 pkg prefix dual_sharpa_wave`、ROS distro、Git checkout；如 Sharpa repo 不在 workspace，再 clone 到 `src/dual_sharpa_wave_ros2`。不能只因 GitHub 的檔案存在，就假設本機版本及依賴已就緒。
2. **建立 `ament_python` package。** `package.xml` 加 `rclpy`、`sensor_msgs`、`std_msgs`、`launch`、`launch_ros`、`ament_index_python`、`xacro`、`python3-yaml`、`joint_state_publisher`、`robot_state_publisher`，以及 `dual_crx_control`、`dual_sharpa_wave` 執行依賴；`setup.py` 設 `hold_and_test = bimanual_manipulation.hold_and_test:main` 並安裝 `launch/`、`config/`。初版不做自訂 message、MoveIt、同步 action server 或 GUI。
3. **實作一個 node、三個狀態訂閱、三個命令 publisher。** 訂閱 `/teleop/joint_states` 和左右手 `joint_states`；發布一個 12 值 CRX 命令及兩個完整 22 值的 Sharpa 命令。依 `name` 將 feedback 排成固定順序，拒絕重名、缺名、長度不符、NaN/Inf。每次發布前確認三路 feedback 都新鮮、命令 subscriber 存在；若缺資料，記錄原因且不要進入動作階段。
4. **初次接通先做 hold。** 收到三路有效 feedback 後，複製當前實際位置作為四設備的目標，以 100 Hz 發布，先只觀察命令及 feedback；啟動時不跳到預設 pose、不自動開合、不自動移動手臂。用 monotonic time 檢查 feedback 年齡，門檻做成參數（例如先用 0.5 秒，再按實際頻率調整）。失去新鮮 feedback 時停止發新目標並報錯；現有底層會怎樣保持最後目標需由操作者知悉，不將其描述成急停。
5. **可選的最小動作。** 明確 CLI/參數啟用時才動作：以 feedback 建立基準，先只改一隻 Sharpa 的一個關節很小的弧度量，其餘 43 個手指關節與 12 個手臂關節保持量測值，回到基準；再選擇做兩手同時的小動作。發布前從 Sharpa URDF 讀取該關節 limits 或使用已驗證的 [`control_model.py`](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/control_model.py) 限制，不可直接猜測上下界。兩臂要做實體小動作時才另行加入 `--move-arms`，仍以當前 feedback 為起點，並在現場確認速度、工作空間、碰撞與 FANUC 授權狀態。
6. **建立整機 description 與總 launch。** 按第 4、5 節合併單一 URDF/TF tree，並用 `IncludeLaunchDescription` 引入兩個現有 launch。預設 CRX 與 Sharpa 都是 mock；總 launch 不自動啟動 `hold_and_test`，也不送動作命令。

## 4. 整機 description：把手真正接到手臂模型

已檢查的現有模型：[CRX `dual_crx.urdf.xacro`](https://github.com/MSC-UCB/dual_crx_control/blob/main/urdf/dual_crx.urdf.xacro) 以 `world` 作根，已包含左右 CRX、基座安裝位姿、`left_ee_mount`、`right_ee_mount`，以及既有的 `left_tcp`、`right_tcp`；[左手 URDF](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/urdf/left_sharpa_wave_with_flange.urdf) 和 [右手 URDF](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/urdf/right_sharpa_wave_with_flange.urdf) 的根 link 分別為 `left_hand_flange`、`right_hand_flange`。兩側手指 joint 名稱已帶 `left_`/`right_` 前綴。這裡的 `ee_mount` 是 CRX 模型的末端接點；實機轉接板尺寸和安裝方向**尚未驗證**。

| 新增的 fixed joint | parent link | child link | `origin` |
| --- | --- | --- | --- |
| `left_arm_to_hand` | `left_ee_mount` | `left_hand_flange` | 左手安裝座 `xyz/rpy`，來自設定檔 |
| `right_arm_to_hand` | `right_ee_mount` | `right_hand_flange` | 右手安裝座 `xyz/rpy`，來自設定檔 |

```text
world
├── left CRX → left_ee_mount → left_hand_flange → 左手各關節
└── right CRX → right_ee_mount → right_hand_flange → 右手各關節
```

最小實作是在 `composite_description.py` 呼叫 `xacro.process_file()` 展開**已存在**的 CRX xacro，再用 XML 解析器讀取左右手的完整 URDF，將手部有效的 link、joint（及其必要的頂層 material 等 URDF 元素）併入同一個 `<robot>`，最後各加一個 fixed joint。不要用 `<xacro:include>` 直接包完整 Sharpa URDF，因為每份都有自己的 `<robot>` 根；也不要在同一個 frame 建兩份 parent joint。依已安裝 package share 找路徑，保留手部 `package://dual_sharpa_wave/...` mesh URI。寫入前檢查 link/joint 無重名、整體只有一個 root、總共 12 個手臂加 44 個手部可動 joint、兩個 mount 端點都存在；用 `check_urdf` 和 mock RViz/TF 驗證。

`config/hand_mounts.yaml` 的兩側安裝座 `xyz/rpy` 先標註為**僅供 mock 檢查的暫定值**；實機需要核對 CRX flange 與 Sharpa flange 的座標方向、轉接板厚度，才可當作真實運動規劃的 TF。既有 `left_tcp`/`right_tcp` 的 35 mm 偏移仍是 CRX 舊模型中的 TCP，不能直接當成 Sharpa 掌心或指尖 TCP；需要抓取規劃時再定義新的 tool frame。

啟動 `bimanual_description.launch.py` 時：

- 一個 `joint_state_publisher` 用 `source_list` 訂閱 `/teleop/joint_states`、`/sharpa/left_hand/joint_states`、`/sharpa/right_hand/joint_states`，輸出 `/bimanual/joint_states`，設 `publish_default_positions: false`；不得將輸出再放回 `source_list`。此合併流用於視覺化／TF，**不能**讓運動安全檢查用它掩蓋個別 feedback 失聯。
- 一個整機 `robot_state_publisher` 接收同一份整機 URDF，訂閱 `/bimanual/joint_states` 並發布全域 `/tf`、`/tf_static`。整機 URDF 與 joint-state merger 使用相同 description。確認左右 arm driver 各自私有 TF 仍留在原命名空間，不額外在全域 `/tf` 發布同名 frame。

## 5. 總 launch：啟動兩套既有控制系統

`bimanual_system.launch.py` 以 `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` 引入 [`dual_arm_teleop.launch.py`](https://github.com/MSC-UCB/dual_crx_control/blob/main/launch/dual_arm_teleop.launch.py) 與 [`dual_sharpa.launch.py`](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/launch/dual_sharpa.launch.py)，再 include `bimanual_description.launch.py`。以下是**待實作的骨架**，安裝 package 檔案與 description launch 後才能執行：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_source(package, filename):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', filename])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('crx_mock', default_value='true'),
        DeclareLaunchArgument('sharpa_backend', default_value='mock'),
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.2.100'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.1.100'),
        IncludeLaunchDescription(
            launch_source('dual_crx_control', 'dual_arm_teleop.launch.py'),
            launch_arguments={
                'mock': LaunchConfiguration('crx_mock'),
                'rviz': 'false',
                'record': 'false',
                'method': 'linear',
                'input_rate_hz': '100.0',
                'left_robot_ip': LaunchConfiguration('left_robot_ip'),
                'right_robot_ip': LaunchConfiguration('right_robot_ip'),
            }.items(),
        ),
        IncludeLaunchDescription(
            launch_source('dual_sharpa_wave', 'dual_sharpa.launch.py'),
            launch_arguments={
                'backend': LaunchConfiguration('sharpa_backend'),
                'use_rviz': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            launch_source('bimanual_manipulation', 'bimanual_description.launch.py')
        ),
    ])
```

關閉兩個原有 launch 的全域 RViz／preview publisher，是為了只由整機 description 發布同一棵全域 TF tree；CRX 的左右 driver 私有 state publisher/controller 仍需保留。總 launch 會啟動 CRX controller 和左右 Sharpa node，但 **launch 不會等待四個設備都 ready，也不會自己啟動運動**。`hold_and_test` 保持獨立命令來源，依第 3 節檢查各路新鮮 feedback 後才允許送命令。不得同時啟動另一套 `dual_arm.launch.py` 或 Sharpa GUI/sine 命令來源。

## 6. 初版啟動與驗收

以下假設 `~/ws_fanuc/src` 中已各有一份兩個 package，FANUC driver 與描述已可解析，並使用 ROS 2 Jazzy。實作時依第 3 節先核對環境，再建 package、編譯：

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to bimanual_manipulation
source install/setup.bash
ros2 pkg prefix dual_crx_control
ros2 pkg prefix dual_sharpa_wave
ros2 pkg prefix bimanual_manipulation
```

兩個 terminal 各自 `source /opt/ros/jazzy/setup.bash`、`source ~/ws_fanuc/install/setup.bash`，並使用相同 `ROS_DOMAIN_ID`。先在 Terminal 1 啟動總 launch，預設均為 mock：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=true sharpa_backend:=mock
```

```bash
# Terminal 2：確認三路 feedback、合併狀態與 TF 後，再手動啟動測試 node
ros2 topic echo /bimanual/joint_states --once
ros2 run tf2_ros tf2_echo world left_hand_flange
ros2 run bimanual_manipulation hold_and_test
```

必要時可將兩個原有 launch 分別開在獨立 terminal 除錯；不要在總 launch 已經啟動時再開一次。

驗收：合併後有單一 `world` 根，左右 `ee_mount → hand_flange` 均可由 TF 查詢，56 個可動關節能更新而沒有重複 frame；`ros2 topic info /teleop/joint_command -v` 和兩個 `/sharpa/*_hand/joint_command` 都有一個控制來源；`ros2 topic echo /teleop/joint_states --once`、左右手 `joint_states --once` 均有回饋；node 回報三路 feedback 已就緒、雙臂和雙手的 target 皆等於初始量測值；mock 中明確開啟小幅動作模式後，指定關節有跟隨且左右手不串線。

實機階段**明確**以 `crx_mock:=false sharpa_backend:=sharpa_sdk` 啟動，並核對左/右臂 IP、手的 serial、Ethernet、ROS 網域及現場安全條件。Sharpa 啟動 terminal 需設定 SDK `PYTHONPATH=/opt/sharpa-wave-sdk/python` 和 `LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib`；細節見 [Sharpa README](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/README.md)。CRX teleop launch 啟動的是可接管動作的 controller，不適合在需要 TP jog/手導的階段同時啟用。先完成 mock 驗收，再在實機上單側、單關節、小幅確認；不要把 mock 成功視為實體無碰撞證明。

## 7. 原始碼入口

- [CRX teleop launch 與參數](https://github.com/MSC-UCB/dual_crx_control/blob/main/launch/dual_arm_teleop.launch.py)、[teleop bridge](https://github.com/MSC-UCB/dual_crx_control/blob/main/src/dual_crx_control/teleop/bridge.py)、[interpolator](https://github.com/MSC-UCB/dual_crx_control/blob/main/src/dual_crx_control/interpolation/node.py)。
- [Sharpa launch 與 namespace](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/launch/dual_sharpa.launch.py)、[hand node](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/hand_node.py)、[joint names](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/dual_sharpa_wave/joint_names.py)、[hardware YAML](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/config/dual_sharpa_hardware.yaml)。
- [CRX 整機 URDF](https://github.com/MSC-UCB/dual_crx_control/blob/main/urdf/dual_crx.urdf.xacro)、[Sharpa 左手 URDF](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/urdf/left_sharpa_wave_with_flange.urdf)、[Sharpa 右手 URDF](https://github.com/MSC-UCB/dual_sharpa_wave_ros2/blob/main/urdf/right_sharpa_wave_with_flange.urdf)。
