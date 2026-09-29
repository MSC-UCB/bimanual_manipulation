# Bimanual manipulation: first mock integration

This ROS 2 Jazzy package combines the local `dual_crx_control` and
`dual_sharpa_wave` packages. The system launch starts both existing control
stacks and publishes one full robot description/TF tree. It sends **no motion
command**. Run `hold_and_test` separately to hold all four devices at their
initial measured positions.

The checked-out CRX interface is `/crx5ia/joint_targets` (`JointState`), not
the older `/teleop/joint_command` interface mentioned in the investigation
plan. The hand topics remain `/sharpa/{left,right}_hand/joint_command`.

## Preview

The three source directories are linked into `~/ws_fanuc/src`. In a WSL Bash
terminal:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  dual_crx_control dual_sharpa_wave bimanual_manipulation
source install/setup.bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=true sharpa_backend:=mock rviz:=true
```

RViz reads `/bimanual/robot_description` with `world` as its fixed frame.
Close the launch with Ctrl+C. In a second, similarly sourced terminal, an
optional hold-only command source is:

```bash
ros2 run bimanual_manipulation hold_and_test
```

Only start one command source for the arm and hand command topics. The hold
node waits for fresh feedback from each arm and hand, then publishes their
measured positions at 100 Hz. It latches a fault and stops publishing if
feedback or a command subscriber disappears. Stopping publication is not an
emergency stop; the existing controllers may retain their last target.

## Edit the model

[`config/hand_mounts.yaml`](config/hand_mounts.yaml) contains the fixed
`left_ee_mount` → `left_hand_flange` and `right_ee_mount` →
`right_hand_flange` transforms. Its current zeros are **mock preview
placeholders**. Edit the `xyz` values in metres and `rpy` values in radians,
then restart the launch. You can pass another file with
`hand_mounts:=/absolute/path/to/hand_mounts.yaml`.

The composite URDF is built at launch from the installed CRX xacro and the
two installed Sharpa URDFs. It has 56 movable joints. The original CRX
arm-only TF is isolated under `/crx5ia/arm_only_tf*`; only the composite
state publisher writes the global `/tf` and `/tf_static` topics.

The mount values and CRX model's existing `left_tcp`/`right_tcp` are not
calibrated hand/tool frames. This first version is for preview and hold in
mock mode; it has no small-motion test or grasp planner yet.
