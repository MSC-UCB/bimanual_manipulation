# 双臂双手集成：需求与实施方案

目标：让 `/home/msc-crx/ws_fanuc/bimanual_manipulation` 能用于真实机器人。

## 原始需求

1. 整合 `dual_crx_control` 与 `dual_sharpa_wave`，使用一个 launch 文件启动双臂和双手。
2. 提供一个统一脚本，让双臂和双手回到默认位置，复用以下两个现有脚本：
   - `/home/msc-crx/ws_fanuc/src/dual_crx_control/scripts/move_to_default_pose.py`
   - `/home/msc-crx/ws_fanuc/src/dual_sharpa_wave_ros2/script/move_to_default_pose.py`
3. 按两个原包的当前实现更新集成项目，尤其核对命名空间和前缀。
4. 提供一个 Python 可视化窗口，显示双臂的 12 根力/力矩热度条，以及双手十指的 10 张触觉热力图；触觉仅显示热力图，参考 `ros2 run dual_sharpa_wave tactile_viewer.py --force-max 15`。
5. 驱动应继续读取两个原包的配置；若需要新增配置，应在方案中明确说明。
6. 尽量减少改动，不增加不必要的约束和限制。
7. 设备均已连接，可以测试，但不得向真机发送运动命令。

## 实施方案（实施前记录）


以下方案保留实施前的分析和步骤；后续实现已完成，当前行为与验证结果以集成包的 README 和验证记录为准。

1. **沿用现有两个包，集成包只负责统一入口。**
   保留 `dual_crx_control` 和 `dual_sharpa_wave` 各自的驱动、控制接口及配置，通过现有 `bimanual_system.launch.py` 统一启动。注意手部源码目录叫 `dual_sharpa_wave_ros2`，实际 ROS 包名是 `dual_sharpa_wave`。不重写驱动、不增加新的控制框架；仅在上游缺少必要参数入口时做小改动。

2. **更新 launch，并逐项核对 namespace、关节名和 TF。**
   当前集成 launch 仍默认使用旧 IP `192.168.10.100/200`，而机械臂 launch 的实际默认值已是左臂 `192.168.2.100`、右臂 `192.168.1.100`；集成层还固定了 `method=linear`。将这些设置改为沿用上游默认值，并允许从统一 launch 显式覆盖插值方式、输入频率及 IP。手部透传 `backend`、`config_file`、`publish_rate_hz`、`read_only` 等现有参数，避免上层覆盖 YAML 中未要求修改的设置。
   机械臂继续使用 `/crx5ia/left`、`/crx5ia/right` 和 `left_J1..J6`、`right_J1..J6`；手部继续使用 `/sharpa/left_hand`、`/sharpa/right_hand`，关节名直接复用 `dual_sharpa_wave.joint_names`。ROS namespace 与 URDF 关节前缀分别处理，不额外给关节名叠加 `hand_` 或 namespace。
   继续复用两个包的最新模型生成组合 URDF，核对 12 个机械臂关节、44 个手部关节、法兰连接和四路 joint_states。检查现有 TF remap，使组合模型负责全局 TF，避免重复发布相同坐标系。

3. **增加一个统一回默认位脚本。**
   在集成包提供 `move_to_default_pose.py`，优先以轻量封装调用两个现有回默认位脚本，复用目标位置、轨迹生成、反馈检查和退出状态。调用使用 ROS 安装入口，避免写死源码绝对路径；若原脚本尚未安装为可执行文件，仅补充安装声明。
   最小实现采用一个命令依次完成“双臂回默认位、双手回默认位”，任一步失败便不进入下一步，退出时清理子进程。这里的“统一”指统一操作入口，不要求四个设备严格同步到达。机械臂目标沿用 `dual_crx_control/config/initial_pose.yaml`，双手沿用现有脚本的各 22 关节零位。
   保留原脚本不带 `--execute` 时只预览、带 `--execute` 时执行的行为，不增加额外确认流程或新的限速层。启动系统本身不自动回默认位。

4. **增加一个双臂力/力矩与双手触觉的 Python 可视化窗口。**
   在集成包提供 `tactile_wrench_viewer.py`，使用现有 OpenCV、NumPy 和 ROS 订阅机制，复用手部 `tactile.from_image` 与 `tactile_viewer.force_heatmap`，不另建 GUI 框架。窗口上部左右各显示 6 根热度条：`Fx/Fy/Fz/Tx/Ty/Tz`，共 12 根；下部按左右手两行、每行五指显示 10 张触觉热力图，不显示 RAW 或 DEFORM 图。
   wrench 订阅 `/crx5ia/{left,right}/force_torque_sensor_broadcaster/wrench`，保留数值正负号，以零点为中心显示，力标注 N、力矩标注 N·m，分别设置显示量程，超量程时保留实际数值。触觉只订阅 `/sharpa/{left,right}_hand/tactile/<finger>/dist_force`，沿用现有查看器对三维力向量取模的算法，默认使用 `--force-max 15`，十张图共享固定色标，单位按现有 SDK 标注，不擅自转换为 N。
   无数据、过期数据和无效数据直接在对应区域标明。查看器只订阅 ROS 数据，不连接 SDK、不发布控制命令；可单独运行，也可由统一 launch 的可选参数启动。

5. **配置继续以两个原包为来源，暂不新增 YAML。**
   机械臂控制器配置继续使用 `dual_crx_control/config/dual_arm_controllers.yaml`，默认位继续使用 `config/initial_pose.yaml`；IP 和插值参数沿用机械臂 launch 的参数入口。手部真实硬件使用 `dual_sharpa_wave/config/dual_sharpa_hardware.yaml`，mock 使用 `dual_sharpa_wave.yaml`，保留序列号、触觉开关、发布频率及 SDK 控制参数。
   集成包只保留已有的 `config/hand_mounts.yaml`。其当前值是零变换占位；若要准确显示手部安装姿态，需要填入实测的左右法兰安装变换，无须为此新建配置。可视化量程先用命令行参数提供。通过 ROS package share 查找配置文件，修改源码配置后按现有构建/安装方式更新，验证运行时确实加载了对应版本。

6. **先离线和 mock 验证，再进行真机只读检查。**
   检查构建、安装入口、launch 参数解析、组合 URDF 和配置引用；在隔离的 ROS domain 中用 mock 验证统一启动、回默认位脚本和异常退出，避免测试发布者接入真机。用合成 wrench/触觉数据核对 12 根热度条、10 张热力图、色标以及断流显示。
   真机阶段先检查已有节点、话题和驱动实例，优先订阅现有 joint_states、wrench、tactile 数据，核对左右对应、消息类型、频率与界面。若必须新启驱动，先确认其初始化不会发送运动指令，再使用现有只读入口，例如手部 `read_only:=true`；机械臂只读 launch 也需先核查底层启动行为，不能只凭文件名判断。
   真机不运行回默认位的 `--execute`，不运行 `hold_and_test`，不发布 joint_targets、joint_command 或任何保持位置命令。真实运动效果留待后续获准的运动测试验证，本阶段只报告实际完成的只读验证结果。

7. **交付内容与完成标准。**
   更新统一 launch、必要的模型/前缀适配、两个 Python 入口、安装依赖和 README。README 给出统一启动、默认位预览/执行、可视化和只读检查命令，并列明实际使用的配置路径。
   完成标准：一个 launch 能组合启动双臂与双手；一个脚本能在 mock 中完成四个设备回默认位；一个窗口显示 12 根 wrench 热度条与 10 张触觉热力图；真机已有数据可被正确读取，整个验证过程不向真机发送运动命令。

## 当前实现说明

实现与操作方法见 [中文使用说明](../README.md)，测试结果见 [验证记录](validation.md)。机械臂默认采用 `quintic` 五次多项式插值；Fx、Fy、Fz 热度条默认量程为 ±10 N。
