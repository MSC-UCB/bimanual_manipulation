# Unified Dual-Arm and Dual-Hand Launch and Sensor Viewer

This package combines `dual_crx_control` and `dual_sharpa_wave`, providing a system launch file, a default-pose script, and a force/torque and tactile viewer. The original packages provide the drivers. The combined URDF contains 56 movable joints.

## Build

Build from `/home/msc-crx/ws_fanuc`. The `bimanual_manipulation` source is in `src/bimanual_manipulation`; colcon discovers it when run from the workspace root.

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  sharpa_control_interfaces dual_crx_control dual_sharpa_wave bimanual_manipulation
source install/setup.bash
```

Source ROS and the workspace in every new terminal. Sharpa hardware also requires the installed SDK's Python modules and shared libraries:

```bash
export PYTHONPATH=/opt/sharpa-wave-sdk/python:${PYTHONPATH}
export LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib:${LD_LIBRARY_PATH}
```
You can add this to bash.rc for covenience.
## Unified Launch

The validated hardware configuration is the default:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py
```

This is equivalent to `crx_mock:=false sharpa_backend:=sharpa_sdk mit_kp_ratio:=0.6 mit_kd_ratio:=0.8 collision_force_threshold_n:=50.0 viewer:=true wrench_force_max:=20 wrench_torque_max:=3`.
`wrench_force_max` and `wrench_torque_max` only change the viewer's display ranges; `collision_force_threshold_n` controls the CRX stop threshold.
Explicit arguments override the defaults. Hand YAML values are used for MIT gain ratios only when the corresponding launch arguments are left empty.

Launch hardware in read-only mode with the combined model and sensor window:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk read_only:=true \
  rviz:=true viewer:=true
```

`read_only:=true` sets FANUC `motion_control` to `0` for both arms and starts only the joint-state and wrench broadcasters, without position controllers or interpolation nodes. The hands use the upstream `read_only` mode, without joint-command subscriptions or SDK motion-control configuration. This mode still establishes device connections and streams state data.

Use normal control mode to run motion scripts afterward:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk rviz:=true viewer:=true
```

Normal control mode starts the underlying motion controllers. The launch does not automatically run the default-pose script. Do not run both modes simultaneously or start duplicate drivers for devices already in use.

Offline mock mode:

```bash
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=true sharpa_backend:=mock rviz:=true viewer:=true
```

Use the same isolated ROS domain in all mock test terminals; use the devices' existing ROS domain for hardware terminals. The current mock hands do not generate tactile data, and mock wrench values may be NaN. The viewer displays `NO DATA` / `INVALID DATA` accordingly, without fabricating sensor values.

| Argument | Default behavior |
| --- | --- |
| `crx_mock` / `sharpa_backend` | `false` / `sharpa_sdk` |
| `read_only` | `false`; `true` disables motion control for both arms and hands |
| `left_robot_ip` / `right_robot_ip` | Empty; inherits the arm launch defaults, currently `192.168.2.100` / `192.168.1.100` |
| `method` / `input_rate_hz` | Empty; inherits the arm launch defaults, currently `quintic` / `100.0` |
| `ruckig_target_mode` / `ruckig_target_timeout` | Empty; inherits the arm launch defaults |
| `collision_force_limit` / `collision_force_threshold_n` | Enabled / `50.0` N |
| `sharpa_config` | Empty; uses the MIT YAML for hardware and the mock YAML for mock mode |
| `mit_kp_ratio` / `mit_kd_ratio` | `0.6` / `0.8`, shared by both hands |
| `sharpa_publish_rate_hz` | Empty; keeps the publication rate from the hand YAML |
| `sharpa_read_only` | `false`; enables read-only mode for the hands independently; global `read_only=true` takes precedence |
| `hand_mounts` | This package's `config/hand_mounts.yaml` |
| `rviz` / `viewer` | `false` / `true` |
| `force_max` | Tactile heatmap upper limit: `15`, in SDK units |
| `wrench_force_max` / `wrench_torque_max` | Bar display ranges: ±`20` N / ±`3` N·m |

When `method` is omitted, the launch inherits `dual_crx_control`'s `quintic` (fifth-degree polynomial) interpolation, with a default target input rate of 100 Hz and interpolation output rate of 500 Hz. Hardware hands default to MIT control with SDK position interpolation disabled (`interpolation: false`); the CRX arms retain their existing position-control path.

Example overrides include `method:=linear input_rate_hz:=100` and `sharpa_config:=/absolute/path/hands.yaml`. List all arguments with:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py --show-args
```

### MIT Control for Both Sharpa Hands

The unified launch defaults to MIT control for hardware hands, without an additional configuration argument:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk \
  read_only:=false sharpa_read_only:=false \
  rviz:=true viewer:=true
```

Before launching, stop existing hand/arm drivers and release Pilot control of the same devices to avoid duplicate connections.
This starts hardware control for both arms and MIT control for both hands. MIT settings apply only to Sharpa; CRX retains its existing position-control path.
At MIT startup, Kp/Kd are written once as the fixed tuning baseline multiplied by `mit_kp_ratio` / `mit_kd_ratio` (0.6 / 0.8 by default in this launch), then read back for verification. Matching values are left unchanged. The driver then sets the mode and enables the hand motors.
Use `mit_kp_ratio:=0.6 mit_kd_ratio:=0.8` to set the startup Kp/Kd ratios independently. Both hands use the same ratios, and restarting does not compound them. A GUI ratio of 1 always corresponds to the same original tuning baseline.
Both hands' logs should show `control_mode=mit, read_only=False`.

Custom configurations can still be selected with `sharpa_config`. To use the original POSITION mode, specify
`sharpa_config:=/home/msc-crx/ws_fanuc/src/dual_sharpa_wave_ros2/config/dual_sharpa_hardware.yaml`.

The MIT YAML sets `interpolation:false`, 100 Hz state publication, and a 0.5 s command-idle threshold for both hands.
The existing retargeting ROS command interface is unchanged: 20 Hz solving, 100 Hz publication, and 50 ms upstream interpolation remain supported.
The state publication rate does not automatically generate motion commands. If valid commands stop for more than 0.5 s, the hand session and feedback remain active.
When commands resume, control restarts from the latest feedback, retaining the check that the first target is within 0.1 rad of feedback.
Normal completion of the homing script or switching teleoperation no longer requires a driver restart. Stopping publication does not disable the motors; the last MIT target may remain active. Actual SDK faults still close and lock the session.
Additional local documentation, if retained, is available in `../dual_sharpa_wave_ros2/docs/mit_control.md`; it is not tracked in Git.

MIT now clamps command targets to the URDF limits. If measured feedback exceeds a limit by more than 5°, normal commands are locked while the connection and feedback remain active; the hands do not automatically return to zero.
The existing `move_to_default_pose.py --execute` command requests hand homing recovery through a dedicated action. Successful recovery clears the lock without restarting the driver.
Cancellation, heartbeat loss, or failure to reach the target ends recovery. Once a recovery has started, its completion leaves normal commands locked only if measured feedback still exceeds a limit by more than 5°; otherwise normal control resumes.
Homing can be requested again. Only worsening limit violations during recovery or SDK faults close the SDK session.
Build the added `sharpa_control_interfaces` package together with the others and source the workspace again. Restart any running driver once to load the updated version.

## Dual-Arm Collision Force Limiting

The system launch also starts the `dual_crx_control` limiter; no separate node launch is needed:

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  crx_mock:=false sharpa_backend:=sharpa_sdk \
  collision_force_limit:=true collision_force_threshold_n:=50.0
```

If `|Fx|`, `|Fy|`, or `|Fz|` on either CRX arm exceeds the threshold, both arms stop and Stream Motion is disabled.
Force checks are initially skipped for 3 s after both drivers and valid force feedback are ready. Wait until `/crx5ia/collision_force_limiter/state` reports `ARMED` before moving. This interval is not an exemption for default-pose motion, and there is no gate blocking external commands.
Missing, invalid, or stale force feedback (older than 0.2 s) produces a warning without disabling the arms. After arming, the limiter enters `DEGRADED`, continues checking thresholds on sides with valid feedback, and returns to `ARMED` when data recovers.
A stop triggered by a valid force reading above the threshold does not recover automatically. Stop external command sources, remove contact, and restart the launch.
A normal trigger stops only CRX, not the Sharpa fingers. An unexpected limiter exit or stop failure shuts down the entire system launch. The limiter is not started with `read_only:=true`.

For ordinary mock demonstrations without valid force feedback, add `collision_force_limit:=false`. Otherwise, NaN mock wrench values may leave force monitoring waiting indefinitely to arm, although the launch does not shut down due to a readiness timeout. Force-limit tests use separate synthetic wrench data.
This feature stops motion through ROS services; it is not a hardware emergency stop. Hardware stopping latency still needs validation under controlled conditions.

## Return to the Default Pose

Preview targets without creating a motion node or connecting to the SDK:

```bash
ros2 run bimanual_manipulation move_to_default_pose.py
```

With normal control mode already running, move both arms to their default pose, then return both hands to zero:

```bash
ros2 run bimanual_manipulation move_to_default_pose.py --execute
```

The script calls the `move_to_default_pose.py` installed by each original package, preserving their trajectories, feedback checks, and rate limits. It stops if the first step fails. Arm targets come from `initial_pose.yaml`; hand targets are zero for all 22 joints on each hand. Use `--arm-rate 100 --hand-rate 100` to set publication rates; the arm rate should match the launch's `input_rate_hz`. Motion does not include collision planning: check the path first and stop other motion-command sources. Ctrl+C terminates the current subprocess and stops sending new targets; it is not a hardware emergency stop.

The legacy `hold_and_test` remains a standalone motion-command tool. The launch does not start it automatically, and it is not intended for read-only checks.

## Unified Sensor Window

Start the viewer separately when drivers are already running:

```bash
ros2 run bimanual_manipulation tactile_wrench_viewer.py --force-max 15
```

The upper section shows six bars per arm for `Fx/Fy/Fz/Tx/Ty/Tz`, preserving signs and numeric values. The lower section shows five heatmaps per hand, displaying only the magnitude of the three-component `DIST_FORCE` vector. All ten heatmaps share a fixed color scale and use the upstream viewer's algorithm and SDK units. Wrench bars saturate outside their display range, with `!` appended to the value. Set ranges with `--wrench-force-max` and `--wrench-torque-max`.

Missing, stale, and invalid data appear as `NO DATA`, `STALE`, and `INVALID DATA`. Data becomes stale after one second without an update by default; change this with `--stale-after`. The viewer only subscribes to ROS messages: it does not connect to the SDK or publish control messages. Press q/Esc or close the window to exit.

Without a desktop, save one PNG snapshot and a JSON status report with the same base name:

```bash
ros2 run bimanual_manipulation tactile_wrench_viewer.py \
  --snapshot /tmp/bimanual.png --duration 5 --force-max 15
```

`--duration` applies only to snapshot mode. Interactive mode runs continuously.

### Tactile Units and Shear Force

The [official Sharpa manual, §5.5, "6-DOF Force"](https://sharpa-robotics.github.io/sharpa-docs/#5-5-6-dof-force) explicitly states that the resultant fingertip force is measured in N and the resultant torque in N·m. This applies to the six-dimensional resultant force/torque over the entire sensing surface and cannot be directly applied to the heatmap's `DIST_FORCE` values.

At the time of the documentation review, no definition of the units or scaling of the three components per `DIST_FORCE` cell was found in public documentation. The tactile color scale therefore remains in SDK units; `--force-max 15` does not mean a confirmed 15 N.

The current heatmap displays each cell's vector magnitude, `sqrt(x² + y² + z²)`, without a separate shear-force display. ROS messages retain all three components. Extracting local shear force requires confirmation of the coordinate frame and corresponding surface normal. The current ROS driver does not publish the SDK's six-dimensional fingertip resultant force/torque data.

## Configuration, Naming, and Model

Runtime uses the original packages' configurations from their installation directories. After building with `--symlink-install`, check the package paths with:

```bash
ros2 pkg prefix --share dual_crx_control
ros2 pkg prefix --share dual_sharpa_wave
```

| Source | Purpose |
| --- | --- |
| `dual_crx_control/config/dual_arm_controllers.yaml` | Controllers and wrench broadcasters |
| `dual_crx_control/config/initial_pose.yaml` | Default pose and initial mock pose |
| `dual_sharpa_wave/config/dual_sharpa_mit.yaml` | Default hardware MIT configuration, serial numbers, tactile settings, and recovery parameters |
| `dual_sharpa_wave/config/dual_sharpa_hardware.yaml` | Hardware configuration for explicitly selecting the original POSITION mode |
| `dual_sharpa_wave/config/dual_sharpa_wave.yaml` | Mock parameters |
| This package's `config/hand_mounts.yaml` | Fixed transforms from each arm to its hand flange |

The hand source directory is `src/dual_sharpa_wave_ros2`, while the ROS package name is `dual_sharpa_wave`. Arm joints are named `left_J1..J6` / `right_J1..J6`. Hand joints retain the original package's `joint_names(side)` without an additional namespace prefix.

- Arm feedback: `/crx5ia/{left,right}/joint_states`.
- Wrench: `/crx5ia/{left,right}/force_torque_sensor_broadcaster/wrench`.
- Hand feedback: `/sharpa/{left,right}_hand/joint_states`.
- Tactile: `/sharpa/{left,right}_hand/tactile/{pinky,ring,middle,index,thumb}/dist_force`.
- Combined feedback and model: `/bimanual/joint_states`, `/bimanual/robot_description`.

The combined model is generated from the current URDF/Xacro files in the two original packages. TF from the original CRX model is isolated; only the combined model publishes global `/tf` and `/tf_static`. The RViz fixed frame is `world`.

`hand_mounts.yaml` currently contains zero-transform placeholders for `left_ee_mount → left_hand_flange` and the corresponding right-side connection. Enter measured xyz values in meters and rpy values in radians, then restart the launch, to represent the actual mounting pose. The current model does not represent calibrated hand mounts or collision geometry.

## Validation

Offline tests do not start devices:

```bash
python3 -m pytest -q src/bimanual_manipulation/test/test_integration.py
```

Build, mock default-pose, TF, viewer, and hardware read-only validation results are recorded in the local `docs/validation.md`, if retained. That report is not tracked in Git.
