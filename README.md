# 双臂、双手统一启动与传感器显示

本包组合 `dual_crx_control` 与 `dual_sharpa_wave`，提供一个系统 launch、一个回默认位脚本和一个力/力矩与触觉查看器。驱动仍由原包提供，组合 URDF 包含 56 个活动关节。

## 构建

在 `/home/msc-crx/ws_fanuc` 下构建；`bimanual_manipulation` 源码位于 `src/bimanual_manipulation`，colcon 从工作空间根目录运行即可发现它。

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  sharpa_control_interfaces dual_crx_control dual_sharpa_wave bimanual_manipulation
source install/setup.bash
```

每个新终端都要 source ROS 和工作空间。Sharpa 真机还需要当前 SDK 的 Python 模块和动态库：

```bash
export PYTHONPATH=/opt/sharpa-wave-sdk/python:${PYTHONPATH}
export LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib:${LD_LIBRARY_PATH}
```

## 统一启动

已验证的真机配置为默认值，直接启动即可：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py
```

等同于 `crx_mock:=false sharpa_backend:=sharpa_sdk mit_kp_ratio:=0.6 mit_kd_ratio:=0.8 collision_force_threshold_n:=50.0 viewer:=true wrench_force_max:=20 wrench_torque_max:=3`。
`wrench_force_max` 和 `wrench_torque_max` 只调整查看器量程；CRX 停止门槛由 `collision_force_threshold_n` 控制。
显式传入参数仍可覆盖默认值；MIT 倍率留空时才使用手部 YAML 的值。

真机只读启动，显示组合模型与传感器窗口：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk read_only:=true \
  rviz:=true viewer:=true
```

`read_only:=true` 将两臂的 FANUC `motion_control` 设为 `0`，只启动 joint-state 和 wrench 广播器，不启动位置控制器或插值节点；双手使用上游 `read_only` 模式，不创建关节指令订阅或设置 SDK 运动控制参数。此模式仍会建立设备连接和状态流。

需要后续运行运动脚本时，使用正常控制模式：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk rviz:=true viewer:=true
```

正常控制模式会启动底层运动控制器；launch 不自动运行回默认位脚本。两种模式不要同时启动，也不要重复启动已运行的设备驱动。

离线 mock：

```bash
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=true sharpa_backend:=mock rviz:=true viewer:=true
```

mock 的所有测试终端使用同一个隔离域；真机终端使用设备原有的 ROS 域。现有 mock 手部不产生触觉数据，mock wrench 可能是 NaN；查看器对应显示 `NO DATA` / `INVALID DATA`，不会伪造传感器值。

| 参数 | 默认行为 |
| --- | --- |
| `crx_mock` / `sharpa_backend` | `false` / `sharpa_sdk` |
| `read_only` | `false`；`true` 同时关闭双臂、双手运动控制 |
| `left_robot_ip` / `right_robot_ip` | 留空，沿用机械臂 launch；当前为 `192.168.2.100` / `192.168.1.100` |
| `method` / `input_rate_hz` | 留空，沿用机械臂 launch；当前为 `quintic` / `100.0` |
| `ruckig_target_mode` / `ruckig_target_timeout` | 留空，沿用机械臂 launch |
| `collision_force_limit` / `collision_force_threshold_n` | 默认开启 / `50.0` N |
| `sharpa_config` | 留空，真机默认 MIT YAML；mock 使用 mock YAML |
| `mit_kp_ratio` / `mit_kd_ratio` | `0.6` / `0.8`，左右手相同 |
| `sharpa_publish_rate_hz` | 留空，保留手部 YAML 发布频率 |
| `sharpa_read_only` | `false`；可单独启用手部只读，全局 `read_only=true` 优先 |
| `hand_mounts` | 本包 `config/hand_mounts.yaml` |
| `rviz` / `viewer` | `false` / `true` |
| `force_max` | 触觉热力图上限 `15`，SDK 单位 |
| `wrench_force_max` / `wrench_torque_max` | 热度条量程 ±`20` N / ±`3` N·m |

不指定 `method` 时，沿用 `dual_crx_control` 的 `quintic`（五次多项式）插值，默认输入目标频率为 100 Hz、插值输出频率为 500 Hz。双手真机默认 MIT，SDK 位置插值关闭（`interpolation: false`）；CRX 手臂仍使用原位置控制流程。

例如 `method:=linear input_rate_hz:=100` 或 `sharpa_config:=/absolute/path/hands.yaml`。完整参数：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py --show-args
```

### Sharpa 双手使用 MIT 控制

统一 launch 的真机双手已默认使用 MIT，无需额外指定配置：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk \
  read_only:=false sharpa_read_only:=false \
  rviz:=true viewer:=true
```

启动前退出原有双手／双臂 driver，并退出 Pilot 对同一设备的控制，避免重复连接。
这会启动真实双臂控制与双手 MIT 控制；MIT 设置仅作用于 Sharpa，CRX 仍使用原位置控制流程。
MIT 启动先按固定调参基准 × `mit_kp_ratio`／`mit_kd_ratio`（本 launch 默认 0.6／0.8）一次写入 Kp／Kd 并读回确认；已相同则跳过。随后设置模式并使能手部马达。
可加 `mit_kp_ratio:=0.6 mit_kd_ratio:=0.8` 分别调整 Kp／Kd 启动倍率，左右手使用相同的一组值，重启不会累乘。GUI ratio=1 始终对应相同的原始调参基准。
两侧日志应出现 `control_mode=mit, read_only=False`。

自订配置仍可通过 `sharpa_config` 指定。如需原 POSITION 模式，指定
`sharpa_config:=/home/msc-crx/ws_fanuc/src/dual_sharpa_wave_ros2/config/dual_sharpa_hardware.yaml`。

MIT YAML 为两手配置 `interpolation:false`、100 Hz 状态发布和 0.5 秒命令闲置判定。
原 retargeting 的手部 ROS 命令接口不变，20 Hz 求解、100 Hz 发布、50 ms 上游插值仍可使用。
状态发布频率并不自动产生运动命令。有效命令中断超过 0.5 秒后，保留手部 session 与回授；
恢复命令时从最新回授重新接管，沿用首笔目标距回授不超过 0.1 rad 的检查。
正常结束归零脚本或切换 teleop 不再要求重启 driver。停止发布不等于停用马达，最后的 MIT
目标仍可能持续作用；真正的 SDK 故障仍会关闭并锁定 session。
细节见 [MIT 说明](../dual_sharpa_wave_ros2/docs/mit_control.md)。

MIT 现在将命令目标裁切到 URDF 上下限；实际回授超出上下限加 0.5° 时，锁定普通命令，
保留连线与回授，不自动归零。原 `move_to_default_pose.py --execute` 指令会通过专用 action
请求双手归零恢复，成功后解除锁定，无需重启 driver。取消／到位失败保留锁定，可再次
请求归零；恢复中持续加重的超限或 SDK 故障才关闭 SDK。新增的
`sharpa_control_interfaces` 包需一起构建并重新 source，已运行的 driver 需重启一次加载新版。

## 双臂碰撞力限制

系统 launch 会一起启动 `dual_crx_control` 的 limiter，不需要另外启动节点：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk \
  collision_force_limit:=true collision_force_threshold_n:=50.0
```

任一 CRX 手臂的 `|Fx|`、`|Fy|` 或 `|Fz|` 超过门槛，会停止两臂并关闭 Stream Motion。
双侧驱动与有效力回授就绪后先忽略 3 秒，等 `/crx5ia/collision_force_limiter/state`
显示 `ARMED` 再开始运动；这段时间不是回默认位动作的豁免，也没有阻挡外部命令的 gate。
启用后无效力回授或超过 0.2 秒断讯也会触发停止。触发后不自动恢复，先停止外部命令来源、
解除接触再重启 launch。正常触发只停止 CRX，不停止 Sharpa 手指；limiter 意外退出或
停止失败则会关闭整个系统 launch。`read_only:=true` 时不启动 limiter。

普通 mock 演示没有有效力回授时，请加 `collision_force_limit:=false`；否则可能因 mock wrench
为 NaN，一直等待并在 180 秒启动超时后关闭 launch。力限制测试使用独立的合成 wrench。
该功能通过 ROS service 停止，不是硬件急停；实机停止延迟仍需在受控条件下验证。

## 回默认位

只预览目标，不创建运动节点或连接 SDK：

```bash
ros2 run bimanual_manipulation move_to_default_pose.py
```

在已启动的正常控制模式下，执行“双臂回默认位，再双手回零位”：

```bash
ros2 run bimanual_manipulation move_to_default_pose.py --execute
```

它调用两个原包安装的 `move_to_default_pose.py`，保留原有轨迹、反馈检查和限速，第一步失败便停止。机械臂目标来自 `initial_pose.yaml`，双手目标为各 22 个关节零位。可指定 `--arm-rate 100 --hand-rate 100`；机械臂 rate 应与 launch 的 `input_rate_hz` 一致。运动不包含碰撞规划，应先确认路径并停止其他运动指令源。Ctrl+C 会结束当前子进程、停止新目标发送；这不等于硬件急停。

旧的 `hold_and_test` 仍保留为独立的运动命令工具，不由 launch 自动启动，不用于只读检查。

## 统一传感器窗口

在已有驱动运行时单独启动：

```bash
ros2 run bimanual_manipulation tactile_wrench_viewer.py --force-max 15
```

上部左右臂各 6 根热度条，显示 `Fx/Fy/Fz/Tx/Ty/Tz`，保留正负号和实际数值；下部左右手各 5 张图，仅显示 `DIST_FORCE` 三维向量的模。十张图共用固定色标，沿用上游查看器算法及 SDK 单位。超出 wrench 量程的条形会饱和，数值后显示 `!`。量程可用 `--wrench-force-max`、`--wrench-torque-max` 设置。

缺失、过期和无效数据显示为 `NO DATA`、`STALE`、`INVALID DATA`；默认一秒无新数据视为过期，可用 `--stale-after` 修改。查看器只订阅 ROS 消息，不连接 SDK，也不发布控制消息。按 q/Esc 或关闭窗口退出。

无桌面时保存一次 PNG 和同名 JSON 状态报告：

```bash
ros2 run bimanual_manipulation tactile_wrench_viewer.py \
  --snapshot /tmp/bimanual.png --duration 5 --force-max 15
```

`--duration` 仅用于截图模式；交互模式持续运行。

### 触觉单位与剪切力

[Sharpa 官方手册 §5.5「6-DOF Force」](https://sharpa-robotics.github.io/sharpa-docs/#5-5-6-dof-force)明确说明，指尖整体合力的单位是 N，整体力矩的单位是 N·m。这一说明针对整个感应表面的六维合力/力矩输出，不能直接套用于热力图的 `DIST_FORCE`。

截至本次查询，公开文档中尚未找到 `DIST_FORCE` 每格三分量的单位与缩放定义。因此触觉色标仍使用 SDK 单位，`--force-max 15` 不表示已确认的 15 N。

当前热力图显示每格三分量的模 `sqrt(x² + y² + z²)`，没有单独显示剪切力。ROS 消息仍保留三个分量；提取局部剪切力需要确认坐标系和对应表面法向。SDK 的指尖六维合力/力矩数据尚未由当前 ROS 驱动发布。

## 配置、命名和模型

没有新增 YAML。实际运行读取安装目录中的原包配置，`--symlink-install` 后可通过包路径核对：

```bash
ros2 pkg prefix --share dual_crx_control
ros2 pkg prefix --share dual_sharpa_wave
```

| 来源 | 用途 |
| --- | --- |
| `dual_crx_control/config/dual_arm_controllers.yaml` | 控制器与 wrench 广播器 |
| `dual_crx_control/config/initial_pose.yaml` | 默认位与 mock 初始位 |
| `dual_sharpa_wave/config/dual_sharpa_mit.yaml` | 默认真机 MIT 配置、序列号、触觉与恢复参数 |
| `dual_sharpa_wave/config/dual_sharpa_hardware.yaml` | 明确选择原 POSITION 模式的真机配置 |
| `dual_sharpa_wave/config/dual_sharpa_wave.yaml` | mock 参数 |
| 本包 `config/hand_mounts.yaml` | 两侧机械臂到手部法兰的固定变换 |

手部源码目录为 `src/dual_sharpa_wave_ros2`，ROS 包名为 `dual_sharpa_wave`。机械臂关节是 `left_J1..J6` / `right_J1..J6`；手部关节直接沿用原包 `joint_names(side)`，不额外添加 namespace 前缀。

- 机械臂反馈：`/crx5ia/{left,right}/joint_states`。
- wrench：`/crx5ia/{left,right}/force_torque_sensor_broadcaster/wrench`。
- 手部反馈：`/sharpa/{left,right}_hand/joint_states`。
- 触觉：`/sharpa/{left,right}_hand/tactile/{pinky,ring,middle,index,thumb}/dist_force`。
- 组合反馈与模型：`/bimanual/joint_states`、`/bimanual/robot_description`。

组合模型从两个原包的当前 URDF/Xacro 生成。CRX 原模型的 TF 被隔离，只有组合模型发布全局 `/tf` 与 `/tf_static`。RViz 固定坐标系为 `world`。

`hand_mounts.yaml` 当前是零变换占位：`left_ee_mount → left_hand_flange` 和右侧对应连接。准确的安装姿态需要填入实测 xyz（米）和 rpy（弧度），再重启 launch；当前模型不能代表已标定的手部安装或碰撞几何。

## 验证

离线测试不启动设备：

```bash
python3 -m pytest -q src/bimanual_manipulation/test/test_integration.py
```

本次构建、mock 回默认位、TF、界面和真机只读结果见 [docs/validation.md](docs/validation.md)。
