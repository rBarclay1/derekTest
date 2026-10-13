# derekTest

ROS 2 software for a Hiwonder JetRover to find a colored object, drive into
pickup position, lift it, return to the robot's starting pose, and place the
object in front of the robot. Home means the robot's position and heading
when a mission is started. The placement point is offset from home by the
arm's reach; it is not the object's original position.

This implementation was prepared without the robot. Fifteen offline Python
control checks pass with ROS interfaces mocked. No ROS build or robot commands
have been run here. Poses, timings, topic availability, and navigation accuracy
still need hardware verification.

**Start here for the next hardware session:**
[Offline mecanum robot guide](docs/offline-robot-guide.md), also available as
[a standalone printable HTML file](docs/offline-robot-guide.html).
It covers transfer, backup/install, stop checks, calibration, staged trials,
troubleshooting, and recording results without internet.

## Mission behavior

```text
INIT → IDLE → PREPARE → SEARCH → ALIGN / APPROACH → PICK
     → VERIFY_PICK → RETURN_HOME → PLACE → COMPLETE

Any active state → ERROR or STOPPED
```

- Launching waits for fresh RGB, depth, odometry, and a successful controller
  readiness service response. It does not move the arm or start searching.
- `/retrieve_object/start` saves home and starts one mission. `auto_start`
  can explicitly enable automatic start after initialization.
- OpenCV identifies red, green, or blue blobs. Several distinct camera frames
  must agree before movement. Tracking uses the same color and nearby pixel
  position rather than switching to the largest blob on every frame.
- Valid, recent depth is required to align and approach. Missing targets stop
  approach immediately. After enough missed frames the robot searches again.
  Missing sensor streams abort the mission. An object closer than the pickup
  range causes an error rather than reversing blindly.
- Pickup requires the target to stay centered in the depth range over distinct
  RGB/depth observations. The arm lifts into a closed-gripper carrying pose.
- Default behavior pauses for an operator to confirm the object is actually
  held. Servo position is not proof of a successful grasp. Set
  `allow_unverified_pick:=true` to return automatically without that check;
  status will continue to report `pick_verified: false`.
- Return uses the saved odometry pose: turn toward home, drive slowly, then
  match the starting heading and settle before placing. The mission completes
  once the placement sequence ends. Physical release remains unverified.
- Each mission runs once. `COMPLETE` permits another explicit start, which
  records a new home. `ERROR` and `STOPPED` require manual recovery and a node
  restart; neither automatically resumes or releases a held object.

## Code map

| Path | Purpose |
| --- | --- |
| `jetrover_retrieve/jetrover_retrieve/retrieve_object_node.py` | Mission state machine, services, sensor freshness, timeouts, status |
| `jetrover_retrieve/jetrover_retrieve/navigation.py` | Planar pose conversion and return velocity calculation |
| `jetrover_retrieve/jetrover_retrieve/color_detect.py` | HSV blobs, nearest-target tracking, depth conversion |
| `jetrover_retrieve/jetrover_retrieve/arm_control.py` | Timer-driven arm actions, pose configuration, pulse limits |
| `jetrover_retrieve/config/retrieve.yaml` | Mission settings and arm calibration values |
| `JetRover-robot-backup/demos/` | Manual driving, camera viewer, browser dashboard |
| `JetRover-robot-backup/ros2_ws/src/` | Backup robot drivers, mapping, and applications |

The root `object_detector_node.py` delegates to the detector package. The
`JetRover/` entry is an empty Git submodule reference with no `.gitmodules`
configuration; this mission does not depend on it. The backup is not a
complete, independently buildable robot image: other installed packages,
including the camera driver and robot description, are required.

## Robot interfaces

| Interface | Default | Type / interpretation |
| --- | --- | --- |
| RGB | `/depth_cam/rgb/image_raw` | `sensor_msgs/Image`, converted to BGR |
| Depth | `/depth_cam/depth/image_raw` | `16UC1` millimeters or `32FC1` meters |
| Odometry | `/odom` | `nav_msgs/Odometry`, fixed parent/child frames during mission |
| Base velocity | `/controller/cmd_vel` | `geometry_msgs/Twist`, meters/sec and radians/sec |
| Arm | `/servo_controller` | `servo_controller_msgs/ServosPosition`, pulse units |
| Controller readiness | `/controller_manager/init_finish` | `std_srvs/Trigger` |
| Mission status | `/retrieve_object/status` | `std_msgs/String`, JSON, published twice/sec and on transitions |

Sensor messages must have increasing, nonzero ROS timestamps matching the
robot clock. Subscriptions use sensor QoS to accept best-effort camera
publishers. RGB and depth must be registered to the same pixels, have the
same resolution, and arrive within `max_rgb_depth_skew_sec`. Equal image
sizes do not prove registration. The included Dabai launch requests depth
registration, but its actual output must be checked on the robot.

The detector's `/detected_object_point` contains **pixel x/y and meter depth**,
not a Cartesian XYZ position. It emits a point only when depth is valid.
The mission processes images directly and does not subscribe to this topic.

## Install later, on the robot

These commands are instructions for later; they have not been executed.
Use the robot's existing ROS environment and installed drivers.

```bash
zsh
source /home/ubuntu/ros2_ws/.robotrc
sudo systemctl stop start_app_node.service
# Copy this repository's jetrover_retrieve/ into ~/ros2_ws/src/.
cd ~/ros2_ws
colcon build --symlink-install --packages-select jetrover_retrieve
source install/local_setup.zsh
```

Also install the updated chassis driver file from
`JetRover-robot-backup/ros2_ws/src/driver/controller/controller/odom_publisher_node.py`
into the matching robot workspace package, preserving the robot's original
file. Rebuild `controller` if its installation does not use symlinks, then
restart the controller. Its new `cmd_vel_timeout_sec` defaults to 0.5 seconds:
commands must continue arriving to keep moving. The mission sends commands
at 20 Hz. This driver timeout also affects manual tools; commands sent only
once will stop after the timeout. Do not overwrite the robot's entire
workspace with the backup.

The mission publishes zero velocity while idle or stopped, so it must have
exclusive base/arm command ownership. Stop the vendor app, dashboard,
teleoperation, and other autonomous controllers before running it.

## Launch and operate later

Start with the arm poses and pickup distances calibrated and a clear route.
The initial controller supports `JetRover_Mecanum` and `JetRover_Tank`; it
rejects `JetRover_Acker` because its search and return depend on turning in
place. Chassis selection defaults to the robot's `MACHINE_TYPE` environment.

```bash
# Launch camera/controllers and the mission; wait in IDLE.
ros2 launch jetrover_retrieve retrieve_object.launch.py target_color:=red

# Use existing camera/controllers instead.
ros2 launch jetrover_retrieve retrieve_object.launch.py start_bringup:=false

# Monitor status in another sourced terminal.
ros2 topic echo /retrieve_object/status

# Save home and begin one mission.
ros2 service call /retrieve_object/start std_srvs/srv/Trigger '{}'

# When status is VERIFY_PICK, confirm only after observing a held object.
ros2 service call /retrieve_object/confirm_pick std_srvs/srv/Trigger '{}'

# Stop at any point; remaining arm steps are cancelled.
ros2 service call /retrieve_object/stop std_srvs/srv/Trigger '{}'
```

For an uninterrupted mission that **does not verify pickup**, explicitly opt
in when launching:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py \
  target_color:=red auto_start:=true allow_unverified_pick:=true
```

Inspect detection separately while camera topics are already running:

```bash
ros2 launch jetrover_retrieve detect_only.launch.py target_color:=red
# Add show_debug_image:=true only with a working graphical display.
```

Pass `config_file:=/absolute/path/retrieve.yaml` to use custom mission and arm
settings. Launch arguments override the corresponding YAML values for
`target_color`, `show_debug_image`, `auto_start`, and `allow_unverified_pick`.
Mission/arm parameters are read-only during node operation; change the
configuration and relaunch to apply calibration changes.

## First-session tools

The robot is the mecanum version. Merge `config/first_session.yaml` into the
full configuration as shown in the offline guide; it selects
`JetRover_Mecanum`, slower speeds, and a one-meter mission radius.

- `mission_mode:=approach_only` skips arm preparation, approaches the object,
  and completes in the pickup band without pickup or return.
- `mission_mode:=return_test` saves home, drives a short forward leg (default
  0.30 m), returns using the normal return controller, and completes without
  arm commands. All sensor freshness, stop, and mission-radius checks apply.
- `mission_mode:=full` runs the normal retrieval mission and is the default.
  The launch argument overrides the YAML `mission_mode` value.
- `ros2 run jetrover_retrieve arm_check --operation ready_pose --execute
  --ros-args --params-file /absolute/path/session.yaml` executes an individual
  calibrated pose with the mission stopped. Other operations: `grasp_pose`,
  `carry_pose`, `place_pose`, `open`, `close`, `ready`, `pick`, `place`.
  Pose-only operations preserve the gripper command. `/arm_check/stop`
  cancels remaining steps; a published move can finish.
- `python3 scripts/robot_preflight.py` checks the installed robot dependencies
  without starting ROS nodes or commanding movement.

Run the offline regression checks from the repository root:

```bash
python3 -B -m unittest discover -s jetrover_retrieve/test -v
```

These checks exercise control logic with mocked ROS interfaces, not ROS
integration, camera processing, driver timing, or physical motion.

## Arm configuration

The default poses are inherited starting points, not calibrated pickup
solutions. Each pose is a flat list of servo ID / pulse pairs for IDs 1–5.
Servo 10 is the gripper and is added separately. Configure `ready_pose`,
`grasp_pose`, `carry_pose`, and `place_pose` in YAML. Placement is at home in
front of the robot, regardless of object color.

Fallback poses are the default. To use saved vendor `.d6a` actions, set
`use_action_groups: true` and configure `action_group_path`, `ready_action`,
`pick_action`, and `place_action`. All three files must exist and have valid
SQLite action rows before a mission starts. A saved pick action must only
pick/lift, not place or release. The configured carrying pose follows it.

Both saved actions and fallback poses pass through the same pulse validation.
Gripper pulses are clamped to the configured range, default 200–700; other
arm pulses must be in 0–1000. The manual dashboard now also limits servo 10
to 200–700. These numerical limits do not establish collision-free poses.

Arm steps advance from the control timer. Stopping cancels future steps;
an already published servo move can finish. A stop does not automatically
open the gripper. Ctrl+C sends zero base velocity before ROS shutdown. The
separate chassis watchdog covers a stalled or crashed mission process only
when the modified driver is installed and still running.

## Limitations and remaining hardware work

- Return follows a direct route to the saved odometry pose. There is no
  obstacle avoidance, route planning, lidar integration, or map localization
  in this mission yet. Use a clear area; a straight return can cross obstacles
  even if the outbound approach did not.
- The backup raw odometry integrates commanded velocities rather than
  measured wheel travel. Its EKF adds IMU input, but does not establish an
  accurate absolute home location. Slippage and calibration errors can move
  the physical drop point. A home marker or mapped localization is a later
  improvement after the first controlled mission.
- Same-color objects can still be confused when they overlap or pass close
  together. The detector recognizes color blobs, not arbitrary object types.
- Camera distance is not a calibrated arm/base coordinate. Tune the pickup
  range and poses for the camera angle, object size, floor, and arm reach.
- Pickup and release are unverified unless an operator confirms pickup. An
  autonomous verification method needs suitable camera/gripper feedback.
- Sensor timeouts stop the mission; there is no automatic recovery/retry
  after an error. Driver process failure or power loss requires a hardware
  stop mechanism; the software watchdog is not independent of the driver.

See `docs/robot-readiness.md` for the deferred hardware checks and development
sequence. No claim of hardware readiness is made by this implementation.
