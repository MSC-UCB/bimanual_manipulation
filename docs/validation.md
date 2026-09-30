# 验证记录：2026-09-29

已完成三个包的构建、70 项离线测试、完整 mock 回默认位流程和真机只读数据检查。真机未运行回默认位脚本，未发布运动目标。

## 构建与离线检查

```bash
colcon build --symlink-install --packages-select \
  dual_crx_control dual_sharpa_wave bimanual_manipulation --cmake-args -DBUILD_TESTING=OFF
python3 -m pytest -q \
  src/bimanual_manipulation/test/test_integration.py \
  src/dual_crx_control/tools/test_move_to_default_pose.py \
  src/dual_crx_control/tools/test_initial_pose.py \
  src/dual_sharpa_wave_ros2/test/test_default_pose.py \
  src/dual_sharpa_wave_ros2/test/test_tactile.py
```

结果：构建成功；`70 passed`。集成测试覆盖：组合模型完整关节名与挂载关系、默认位预览、子步骤失败后不继续、Ctrl+C 清理子进程、正负 wrench 条、触觉共享色标、无效/缺失/过期数据，以及只读硬件模型中的 `motion_control=0`。

确认 package share 配置的符号链接指向当前源码目录，未复制或新建驱动 YAML。`--show-args`、默认位预览和 Python 编译检查通过。

## Mock 验证

测试使用本机隔离的 ROS domain 198，`crx_mock=true`、`sharpa_backend=mock`；先从驱动描述确认使用 `mock_components/GenericSystem`，再给两臂 J1 增加 0.03 rad、两手 index_PIP 增加 0.08 rad。随后停止测试指令源，执行统一回默认位脚本，确认四个设备均回到原包默认目标。

组合 joint_states 包含 56 个关节，全局 `/tf` 只有一个发布者，两侧 hand_flange 的父节点均为对应 ee_mount。最终版本还通过 `viewer=true` 启动检查，GUI 使用 Qt offscreen 模式；独立查看器在 domain 199 接收合成数据，通过 `--snapshot` 和 ROS 节点重命名参数保存 PNG，12 路输入全部为 LIVE。

mock 手部没有触觉生成器，mock wrench 包含非有限值，查看器正确显示 NO DATA / INVALID DATA。这不代表真实传感器状态。记录见 [mock_validation.json](mock_validation.json)，颜色与正负条形样例见 [synthetic_view.png](synthetic_view.png)。

## 真机只读验证

在原有 ROS domain 87 上运行约 30 秒：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk read_only:=true rviz:=false
```

| 检查项 | 结果 |
| --- | --- |
| 两臂 driver description | 均为 `motion_control=0` |
| 两臂活动控制器 | 仅 joint_state_broadcaster、force_torque_sensor_broadcaster |
| 双手 backend | sharpa_sdk，read_only=True，tactile=True |
| 左/右机械臂 joint_states | 各 6 关节，约 499.8 Hz |
| 左/右手 joint_states | 各 22 关节，100.0 Hz |
| 组合 joint_states | 56 关节，100.0 Hz |
| 两路 wrench | 全部 LIVE，6 轴值有限 |
| 十路 DIST_FORCE | 全部 LIVE，60×60×3，约 27.3–29.0 Hz |
| 运动话题 | joint_targets、interpolated_joint_commands、两手 joint_command、两臂 controller commands 的发布者和订阅者全部为 0 |

只读路径复用了 FANUC 的 `motion_control=0` 数据流模式，不启动 RMI 运动程序或位置控制器；底层通信包的运动控制标志为 0。Sharpa 使用现有只读实现，跳过控制模式/速度/电流/控制源设置，不建立命令订阅。

结果和实际读数见 [real_readonly_validation.json](real_readonly_validation.json)，实际传感器画面见 [real_readonly.png](real_readonly.png)。截图由同一个查看器渲染函数生成，触觉固定上限为 15 SDK 单位。

测试退出时 FANUC 底层驱动输出状态流关闭/重复清理警告，并由 launch 在 SIGINT 超时后发送 SIGTERM 完成退出；没有遗留本次测试的硬件驱动进程。本次没有修改底层驱动的退出流程。

## 尚未验证的物理行为

真实机器人回默认位运动未执行，完整执行流程在 mock 中验证。`config/hand_mounts.yaml` 仍是零变换占位，未进行手部安装几何标定；需填写实测值才能准确显示安装姿态。传感器显示已验证实时接收，不等同于力/触觉物理标定。
