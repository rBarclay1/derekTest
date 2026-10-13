# JetRover mecanum: offline first-session guide

Prepared October 13, 2026. Robot: **JetRover_Mecanum**. Keep this guide and the supplied code archive on your laptop or USB drive. The HTML version opens in a browser without internet and can be printed to PDF.

**Current status:** ready for staged hardware checks, not yet proven on the robot. Fifteen local Python control checks passed with ROS interfaces mocked. No robot connection, ROS build, camera test, wheel movement, or physical pickup has been performed here.

**What this mission does:** find a red, green, or blue object, approach it, pick it up, return to the robot's starting position and heading, and place the object in front of the robot. “Home” is saved when you call start. This does not put the object back where it was found. Return follows a direct route without obstacle avoidance.

## 1. Bring these items and follow this order

Bring the laptop and charger, robot and charger, this guide and code archive, a USB drive or known local SSH connection, one small matte red object that fits the gripper, a tape measure, floor tape, and a clear level floor area. Keep the robot's power cutoff accessible. Do not begin with an object that could break if dropped.

| Stage | Run | Continue when |
| --- | --- | --- |
| Installation | Environment, dependency check, backup, build | Required packages load and build succeeds |
| Base | Watchdog and forward/turn/sideways pulses | Correct directions and automatic stop |
| Camera | Detector only, known distances | Stable registered RGB/depth, including pickup range |
| Return | Empty 30 cm outward/return test | Stops at home; physical error is measured and acceptable |
| Arm | One pose at a time, then pickup/place | Reach, grip, lift, and release work while stationary |
| Approach | Approach-only mode | Centers and stops inside calibrated pickup range |
| Full mission | Explicit start and pickup confirmation | Returns and places successfully in repeated trials |

Do these in order. A failed stage is useful information: record it and fix it before combining more actions. If time is short, finish the base, camera, and empty return checks first.

## 2. Stop commands: keep these available

While a mission is running, call this from another sourced robot terminal:

```bash
ros2 service call /retrieve_object/stop std_srvs/srv/Trigger '{}'
```

Expected: `success: true`, status `STOPPED`, base command zero, and no further arm steps. An arm move already sent may finish. Stopping does not open the gripper. `STOPPED` and `ERROR` require restarting the mission process before a new mission.

During a standalone arm check:

```bash
ros2 service call /arm_check/stop std_srvs/srv/Trigger '{}'
```

Ctrl+C in the mission or arm-check terminal also ends that process. For manual base checks, a separate terminal can send zero:

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{}'
```

A zero message is not a latched emergency stop: another publisher can immediately command movement. End the moving publisher too. If software cannot stop the robot, use its physical power cutoff. Support a held object before removing power.

## 3. Copy the files before losing internet

The provided archive is `JetRover-offline-kit.tar.gz`. It contains a folder named `derekTest-offline` with the mission package, this guide, checks, and the two driver files needed below. It contains source code; it does not bundle ROS or the robot vendor's installed dependencies.

Copy the archive onto the robot with a USB drive, or over a local network. Local SSH works without internet if the laptop and robot can reach each other. Replace `ROBOT_IP` with the robot's actual address; it is not a literal hostname.

On the Mac, from the folder containing the archive:

```bash
scp JetRover-offline-kit.tar.gz ubuntu@ROBOT_IP:~/
ssh ubuntu@ROBOT_IP
```

On the robot, extract it:

```bash
cd ~
tar -xzf JetRover-offline-kit.tar.gz
```

For USB transfer, first copy the archive into the robot's home directory with its file manager, then run the same extraction command. If the extracted folder already exists and has your edits, rename it before extraction.

## 4. Set up every robot terminal

Everything after this section runs **on the robot**, not in macOS Terminal unless you have SSH'd into the robot. Use the existing Ubuntu/ROS 2 Humble installation. The repository backup is not a complete robot image.

The saved `.robotrc` sources zsh files. Start zsh once in each robot terminal, then source the robot's own environment:

```bash
zsh
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
export MACHINE_TYPE=JetRover_Mecanum
export JETROVER_REPO="$HOME/derekTest-offline"
export JETROVER_CONFIG="$HOME/jetrover-session.yaml"
```

Use this terminal setup in every new SSH session or tab. If your copy lives somewhere else, adjust `JETROVER_REPO`. Do not replace the robot's `.robotrc` or `.typerc` with the backup. If the robot's installed setup differs, use the environment that previously worked for manual driving, with matching shell setup files.

Check the environment and dependencies:

```bash
printenv ROS_DISTRO MACHINE_TYPE DEPTH_CAMERA_TYPE need_compile ROS_DOMAIN_ID
python3 "$JETROVER_REPO/scripts/robot_preflight.py"
```

Expected: Humble, `JetRover_Mecanum`, and the camera/settings actually installed on the robot. The bundled camera launch expects `DEPTH_CAMERA_TYPE=Dabai` for depth; its USB-camera branch provides no usable depth for this mission. The check lists missing Python modules, ROS packages, and launch paths. It sends no movement commands.

**Offline dependency limit:** source files cannot replace missing ROS/vendor packages. If this check fails, first confirm you sourced the existing robot environment. Resolve remaining missing packages while internet is available, or use the robot's known working installation. A missing `orbbec_camera`, robot description, or servo message package is a setup blocker. Do not reinstall the whole robot image for this project.

## 5. Back up, install, and build

Stop the vendor application, and close the dashboard, joystick/teleop tools, and other autonomous nodes before taking control:

```bash
sudo systemctl stop start_app_node.service
ros2 node list
```

If the service does not exist, identify and stop the vendor application using the method you used for manual driving. Do not run two copies of the controller/camera. This guide starts them separately below so the mission can be restarted without shutting down the drivers.

Make a unique backup of the installed files that will change:

```bash
export JETROVER_BACKUP="$HOME/jetrover-backups/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$JETROVER_BACKUP"
cp -a "$HOME/ros2_ws/src/driver/controller/controller/odom_publisher_node.py" "$JETROVER_BACKUP/odom_publisher_node.py"
cp -a "$HOME/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py" "$JETROVER_BACKUP/servo_controller.launch.py"
if [ -d "$HOME/ros2_ws/src/jetrover_retrieve" ]; then
    cp -a "$HOME/ros2_ws/src/jetrover_retrieve" "$JETROVER_BACKUP/jetrover_retrieve"
fi
printf '%s\n' "$JETROVER_BACKUP"
```

Record that backup path. If these source paths do not exist, locate the installed packages with `ros2 pkg prefix controller` and `ros2 pkg prefix servo_controller`; stop installation until you know the correct workspace. The commands below assume the stock layout shown above.

Compare the original driver to the supplied version if your robot has custom driver changes. Preserve those customizations when applying the command-timeout change. Otherwise install the supplied files:

```bash
mkdir -p "$HOME/ros2_ws/src/jetrover_retrieve"
cp -R "$JETROVER_REPO/jetrover_retrieve/." "$HOME/ros2_ws/src/jetrover_retrieve/"
cp "$JETROVER_REPO/JetRover-robot-backup/ros2_ws/src/driver/controller/controller/odom_publisher_node.py" "$HOME/ros2_ws/src/driver/controller/controller/odom_publisher_node.py"
cp "$JETROVER_REPO/JetRover-robot-backup/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py" "$HOME/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py"
cd "$HOME/ros2_ws"
colcon build --symlink-install --packages-select controller servo_controller jetrover_retrieve
source install/local_setup.zsh
ros2 pkg executables jetrover_retrieve
```

Expected: a successful build, then `retrieve_object`, `object_detector`, and `arm_check`. The odometry driver now stops motion when velocity commands expire, default 0.5 seconds. The servo launch fix adds a missing package-location import used in its compiled-install branch. Restart already running controllers after installation. Do not copy the whole backup workspace over the robot.

Create one editable session configuration by merging the slower mecanum settings into the full configuration. This refuses to overwrite an existing session file:

```bash
python3 - <<'PY'
import os
from pathlib import Path
import yaml
root = Path(os.environ['JETROVER_REPO']) / 'jetrover_retrieve/config'
config = yaml.safe_load((root / 'retrieve.yaml').read_text())
overrides = yaml.safe_load((root / 'first_session.yaml').read_text())
config['/**']['ros__parameters'].update(overrides['/**']['ros__parameters'])
output = Path(os.environ['JETROVER_CONFIG'])
with output.open('x') as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
print(output)
PY
```

If it already exists, keep it and inspect it with `nano "$JETROVER_CONFIG"`. Defaults for this first session: maximum forward 0.04 m/s, return 0.03 m/s, yaw maximum 0.2 rad/s, 1 m mission radius, 30 cm empty return trial, and manual pickup confirmation. Arm poses and pickup distance are still uncalibrated starting values.

## 6. Start the drivers and inspect ownership

In sourced **terminal A**, start controllers and leave this running:

```bash
ros2 launch controller controller.launch.py
```

In sourced **terminal B**, start the camera and leave this running:

```bash
ros2 launch peripherals depth_camera.launch.py
```

Do not also use a mission launch with `start_bringup:=true` while these are running. Vendor startup behavior can differ; keep the arm and wheels clear when bringing up drivers.

In sourced **terminal C**, inspect the interfaces:

```bash
ros2 service call /controller_manager/init_finish std_srvs/srv/Trigger '{}'
ros2 param get /odom_publisher cmd_vel_timeout_sec
ros2 topic list -t
ros2 topic info /controller/cmd_vel --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /servo_controller --verbose
```

Expected: controller readiness `success: true`, watchdog `0.5`, camera/depth/odom topics, and no unexplained motion publishers. Both `/controller/cmd_vel` and `/cmd_vel` can command the driver. Before manual pulses or arm checks, the mission must be shut down, since even its IDLE state publishes zero base velocity.

If the watchdog parameter is missing, you are running an older driver or wrong installation. Locate the module with the following command, rebuild the intended package, source its install setup, and restart terminal A:

```bash
python3 -c 'import controller.odom_publisher_node as m; print(m.__file__)'
```

## 7. Check the mecanum base and its stop behavior

Do this with no mission node running. Initially support the chassis securely so the wheels cannot propel it; keep fingers and cables away from wheels. Confirm the command-timeout parameter before proceeding. Then repeat short pulses on the clear floor.

One forward command, which should expire automatically:

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.03}}'
```

Expected: a brief forward wheel response followed by a stop about 0.5 seconds after the last command reaches the driver. Allow a little scheduling delay. If movement persists beyond about a second, stop physically and fix the driver/competing publisher issue before further trials.

Check positive yaw and sideways motion separately:

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.12}}'
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.03}}'
```

Run one command at a time and observe its full stop. Positive `x` should move forward, positive `y` left relative to the robot, and positive `z` should turn counterclockwise viewed from above. The mission uses forward motion and turning; the sideways pulse checks the confirmed mecanum configuration. Wrong movement means wheel arrangement, motor mapping, chassis settings, or signs need fixing before autonomy.

For a longer but bounded forward pulse on the floor:

```bash
timeout --signal=INT 3s ros2 topic pub --rate 20 /controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.03}}'
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{}'
```

`timeout` exit code 124 is expected. Actual motion time is shorter than three seconds because ROS discovery takes time; do not treat this as a precise distance command. After this publisher exits, the watchdog should stop the chassis even if you omit the final zero during a supervised watchdog check. Measure floor travel and compare to odometry. On this driver, odometry integrates commanded velocity; wheel spin while lifted still changes reported position.

## 8. Check camera, depth, and target recognition

Keep the chassis stationary. In terminal C, inspect one message from each stream; `--no-arr` avoids dumping image bytes:

```bash
ros2 topic echo /depth_cam/rgb/image_raw --once --no-arr --qos-reliability best_effort
ros2 topic echo /depth_cam/depth/image_raw --once --no-arr --qos-reliability best_effort
ros2 topic echo /odom --once --qos-reliability best_effort
```

Expected: live nonzero timestamps, equal RGB/depth image dimensions, depth encoding `16UC1` (millimeters) or `32FC1` (meters), and odometry with nonempty fixed frame names. Equal image dimensions do not prove the depth image is registered to RGB.

Check rates one at a time; stop each with Ctrl+C after several seconds:

```bash
ros2 topic hz /depth_cam/rgb/image_raw
ros2 topic hz /depth_cam/depth/image_raw
ros2 topic hz /odom
```

Record actual rates. Every stream must remain fresh within 0.5 s; RGB and depth observations must be within 0.15 s. A stream can have an acceptable average rate yet still pause too long. If a rate command sees nothing but messages exist, inspect QoS with `ros2 topic info TOPIC --verbose` and use the best-effort echo commands above.

Start detector-only operation in terminal C:

```bash
ros2 launch jetrover_retrieve detect_only.launch.py config_file:="$JETROVER_CONFIG" target_color:=red
```

In another sourced terminal:

```bash
ros2 topic echo /detected_object_point
```

For a local robot desktop with a working display, add `show_debug_image:=true` to the detector launch to see contours. Leave it false for headless SSH.

The point reports `x/y` in image pixels and `z` in meters. It is not Cartesian arm coordinates. Place one red object at measured camera distances of approximately 0.60, 0.40, and 0.30 m, then at distances reachable by the arm. Measure from the depth camera, not the front bumper. Move the object left/right against a background at a different distance: the depth should follow the object, not the background. Record tape distance and reported `z`; aim for repeatable readings close enough to select the pickup band reliably. Large discrepancies or background readings fail this stage.

Remove the object: no new points should appear. Cover the depth camera: valid points should stop. Restore it: points should resume. This is an invalid-depth test; it does not stop the camera message stream.

The default pickup band is 0.16–0.28 m. **Do not assume the camera can measure that close.** If depth disappears at reachable pickup distances, stop here and resolve camera placement or calibrated pickup geometry. Increasing the maximum pickup distance only helps if the arm can actually reach the object at that robot position. Do not substitute zero depth or loosen freshness checks to force pickup.

Edit topics, `min_object_area`, or depth settings in `nano "$JETROVER_CONFIG"` as needed; relaunch to apply changes. HSV thresholds currently live in `jetrover_retrieve/color_detect.py`, not YAML. If changing them on the robot, edit the deployed package under `~/ros2_ws/src/jetrover_retrieve`, then restart the detector. Stop detector-only with Ctrl+C before mission trials.

## 9. Empty return trial: measure physical home accuracy

Do this before picking anything up. Mark the base center reference and starting heading on the floor. Leave room for a turn and direct return. The arm should already be in a stable tucked position. It will not receive commands in this mode. Keep controllers and camera running.

In terminal C:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py start_bringup:=false config_file:="$JETROVER_CONFIG" mission_mode:=return_test
```

In sourced **terminal D**, monitor status:

```bash
ros2 topic echo /retrieve_object/status --field data
```

Wait for `IDLE`. In another sourced terminal call start:

```bash
ros2 service call /retrieve_object/start std_srvs/srv/Trigger '{}'
```

Expected: `TEST_OUTBOUND` drives approximately 0.30 m according to odometry, then `RETURN_HOME` turns toward home, drives back, restores the saved heading, and settles into `COMPLETE`. This mode uses the same return controller as the full mission, requires the same fresh sensors, and never starts pickup or placement. It does not need a colored object.

First, interrupt an outward leg with `/retrieve_object/stop` and confirm it stops. Restart terminal C after `STOPPED`, then run a complete trial. If the return direction is wrong or the robot drifts out of the clear area, stop; do not compensate by increasing speed.

Measure the physical base offset and heading error against the tape after `COMPLETE`. Default estimated arrival tolerances are 8 cm and 0.12 rad (about 7 degrees), so some residual offset is intentional. Those are odometry tolerances, not guaranteed physical accuracy. A starting acceptance target is physical error under 10 cm and heading error under 10 degrees for three short trials; tighten this if your placement area needs greater accuracy. Record each result. If errors exceed the usable placement area, do not proceed to automatic placement.

Start each repeat from the marked pose, and call start again only when settled. Each start saves a new home. Do not reset odometry or restart the controller during a trial. Increase distance only after repeatability is established, and keep `return_test_distance_m` below `max_home_distance_m`. At 0.03 m/s, a longer outward leg also needs a longer `return_test_timeout_sec`. Relaunch after configuration edits.

## 10. Calibrate the arm while the base is stationary

Stop the mission process in terminal C first. Leave controller bringup running. Close other arm publishers. Use the vendor manual arm tool you previously used to find comfortable starting poses; the supplied pulses have not been calibrated to this robot.

Edit these fields in the session YAML:

```bash
nano "$JETROVER_CONFIG"
```

`ready_pose`, `grasp_pose`, `carry_pose`, and `place_pose` contain alternating servo IDs and pulse values. Example: `[1, 500, 2, 750, ...]`. IDs 1–5 are the arm. Servo 10 is the gripper, controlled by `gripper_open` and `gripper_closed`. Stay within the configured gripper limits (initially 200–700). These numeric limits do not establish that a pose is collision-free.

After inspecting and adjusting a pose, command it individually. Run **one line at a time**, watching the whole motion before continuing:

```bash
ros2 run jetrover_retrieve arm_check --operation ready_pose --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation open --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation grasp_pose --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation close --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation carry_pose --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation place_pose --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation open --execute --ros-args --params-file "$JETROVER_CONFIG"
```

`*_pose` operations leave the gripper command unchanged. `open` and `close` command only the gripper. Each command requires `--execute`, waits for controller readiness and a servo subscriber, and exits after its timed move. “Complete” means commands finished; inspect whether the arm actually reached the pose.

First do the motions empty. Then put the test object at the calibrated pickup point and repeat grasp → close → carry. It must stay held while lifted. Next use place → open and confirm release without hitting the floor or chassis. Keep hands clear while commands execute. Test `/arm_check/stop` during an empty move; a sent move can finish, but future sequence steps must be cancelled.

Once individual poses work, test the actual sequences separately:

```bash
ros2 run jetrover_retrieve arm_check --operation ready --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation pick --execute --ros-args --params-file "$JETROVER_CONFIG"
ros2 run jetrover_retrieve arm_check --operation place --execute --ros-args --params-file "$JETROVER_CONFIG"
```

Run `ready`, position the object, then `pick`. Observe a stable carry before `place`. Set the final `grasp_distance_min/max` using the detector's measured camera distance when the object is reliably reachable. Keep `use_action_groups: false` for these initial fallback-pose trials. Saved vendor actions require separate validation and can include movements beyond these poses.

## 11. Approach-only check and failure checks

With the arm already positioned as it would be during search, put the red object roughly 0.4–0.6 m ahead in the clear area. Stop the standalone arm tool. Launch:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py start_bringup:=false config_file:="$JETROVER_CONFIG" mission_mode:=approach_only target_color:=red
```

Wait for `IDLE`, monitor status, and call `/retrieve_object/start` as in the return test. Expected: `SEARCH` → `ALIGN`/`APPROACH` → `COMPLETE` once centered and stable inside the calibrated pickup band. This mode sends no arm commands and performs no return. With an object on the left, the robot should turn left to center it; verify both sides in separate short trials.

Run these checks separately, restarting the mission after `STOPPED` or `ERROR`:

| Check | How | Expected result |
| --- | --- | --- |
| Operator stop | Call stop during approach | Immediate zero base command and `STOPPED` |
| Target lost | Remove the target from view | Forward motion stops; after missed frames it may rotate to search |
| Invalid depth | Cover the depth lens while RGB still sees the target | Base holds zero while target depth is invalid; no pickup |
| Too close | Start with an object closer than minimum but still valid depth | `ERROR`; no blind reverse or pickup |
| Camera stream stops | Ctrl+C terminal B during approach | `ERROR` for stale RGB/depth, normally within 0.5 s plus scheduling delay |
| Search timeout | Start without a target in a clear area | Search rotates, then `ERROR` after configured search timeout |

If covering the lens also removes the RGB target, search rotation is expected after target loss. It is a different condition from “visible target with invalid depth.” Stop the mission when you have observed the intended behavior. Restart the camera and confirm fresh streams before the next trial.

For an isolated stream-loss check without disconnecting the camera or stopping the driver, relay a real stream into a temporary topic. For example, with all mission processes stopped, copy your session config to a separate file and point its `odom_topic` to `/commissioning/odom`. Create the copied config and change only `odom_topic` to `/commissioning/odom`:

```bash
cp "$JETROVER_CONFIG" "$HOME/jetrover-odom-loss.yaml"
nano "$HOME/jetrover-odom-loss.yaml"
```

Relay the real odometry using this command in another sourced terminal:

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init()
node = Node('commissioning_odom_relay')
pub = node.create_publisher(Odometry, '/commissioning/odom', qos_profile_sensor_data)
sub = node.create_subscription(Odometry, '/odom', pub.publish, qos_profile_sensor_data)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

Launch the test in terminal C with the copied config:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py start_bringup:=false config_file:="$HOME/jetrover-odom-loss.yaml" mission_mode:=return_test
```

Wait for IDLE, call `/retrieve_object/start`, and Ctrl+C the relay during the outward leg. Expected: stale odometry `ERROR` and zero motion. The real driver and `/odom` keep running. To test RGB or depth alone, use the same relay pattern with `sensor_msgs.msg.Image`, the corresponding real source topic, a `/commissioning/rgb` or `/commissioning/depth` destination, and only the matching topic field changed in another copied config. Keep original message timestamps. Restore the normal config for subsequent trials. Do not kill the low-level controller as a way to test sensor loss.

The backup EKF can continue publishing `/odom` after upstream input loss. Fresh `/odom` alone does not prove healthy raw odometry/IMU. These checks verify the mission's input timeout, not every upstream hardware failure.

## 12. Run one full supervised mission

Only continue once base stops, depth, empty return, arm sequences, and approach checks pass. Stop the preceding mission and restore the normal session config. Mark home and the expected placement area in front of the robot. Position one red object at a known reachable location along a clear route.

In terminal C:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py start_bringup:=false config_file:="$JETROVER_CONFIG" mission_mode:=full target_color:=red allow_unverified_pick:=false
```

Wait for IDLE, then start in another sourced terminal:

```bash
ros2 service call /retrieve_object/start std_srvs/srv/Trigger '{}'
```

Expected sequence:

```text
IDLE → PREPARE → SEARCH → ALIGN / APPROACH → PICK
     → VERIFY_PICK → RETURN_HOME → PLACE → COMPLETE
```

At `VERIFY_PICK`, visually confirm the object is securely held. The first-session configuration allows 120 seconds for this. Only then call:

```bash
ros2 service call /retrieve_object/confirm_pick std_srvs/srv/Trigger '{}'
```

Confirmation immediately permits return. If the grip failed, call stop, support/recover the object manually, and restart after correction. Do not confirm merely because the arm closed.

After COMPLETE, inspect release, drop location, and base position. Repeat at least three times with the same setup before varying object position or lighting. Record failed trials as well as successful ones. Keep manual pickup confirmation enabled until there is a trustworthy automatic pickup-success signal. `allow_unverified_pick:=true` bypasses the operator check; it does not add grasp verification.

## 13. Save evidence and finish the session

Before a trial, optionally record a small ROS bag in an extra sourced terminal:

```bash
mkdir -p "$HOME/jetrover-session-logs"
ros2 bag record -o "$HOME/jetrover-session-logs/trial-$(date +%Y%m%d-%H%M%S)" /retrieve_object/status /odom /controller/cmd_vel
```

Stop recording with Ctrl+C after the trial. These topics omit camera images to keep recordings small. If rosbag is unavailable, capture text instead:

```bash
ros2 topic echo /retrieve_object/status --field data | tee "$HOME/jetrover-session-logs/status.txt"
```

Keep a phone video showing the robot and floor marks, and save your calibrated YAML and launch errors. Create a separate result row for each run:

| Trial | Mode | Object camera distance | Physical home offset / heading | Grip / release | Final state / issue |
| --- | --- | --- | --- | --- | --- |
| 1 | return_test | n/a | ___ cm / ___ degrees | n/a | ___ |
| 2 | approach_only | ___ m | n/a | n/a | ___ |
| 3 | full | ___ m | ___ cm / ___ degrees | ___ | ___ |

When finished, stop the mission first, then the camera and controllers. Confirm the base is stopped and remove/support any held object before power-off. If returning to vendor operation, close your test publishers and launches before restarting its service:

```bash
sudo systemctl start start_app_node.service
```

The new watchdog also affects manual tools: they must keep sending commands to keep moving. If you need to restore the original two files, stop all drivers, set `JETROVER_BACKUP` to the recorded backup folder, restore, rebuild, and restart:

```bash
cp "$JETROVER_BACKUP/odom_publisher_node.py" "$HOME/ros2_ws/src/driver/controller/controller/odom_publisher_node.py"
cp "$JETROVER_BACKUP/servo_controller.launch.py" "$HOME/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py"
cd "$HOME/ros2_ws"
colcon build --symlink-install --packages-select controller servo_controller
source install/local_setup.zsh
```

Restoring the original driver removes its new command timeout. Reinstall and recheck the watchdog before running this mission again.

## 14. Quick troubleshooting

| Symptom | Check / next action |
| --- | --- |
| `ros2` or package not found | Source the robot environment and workspace in this terminal; inspect the dependency check |
| `.zsh` setup errors in bash | Start zsh, then source `.robotrc`; do not mix setup-file shell types |
| `KeyError: need_compile` or camera setting | Source the robot's `.robotrc` / `.typerc`; preserve its vendor values |
| Startup timeout | Confirm controller service and all three fresh streams; restart mission after fixing the source |
| Controller says ready but stale sensors | Inspect topic names, ROS domain, timestamps, camera registration, and QoS |
| No detector points | Check color/area, valid depth at object, RGB/depth dimensions, timing, and encoding |
| Object visible but approach does not move | Depth may be invalid or unsynchronized; inspect measured depth before changing thresholds |
| Wrong wheel direction | Check mecanum machine type, physical wheel layout and motor mapping; stop autonomy |
| “Too close” | Stop/restart after manually repositioning; calibrate the actual reachable pickup band |
| At home in status, far away physically | Command-integrated odometry drift or wrong motion scale; measure errors and resolve before placement |
| Grasp misses or collides | Recalibrate poses with a stationary object and compare camera distance to arm reach |
| Arm command completes without motion | Confirm servo controller/subscriber, servo power, and actual driver messages |
| Stop service hangs | Check whether node/ROS connection is alive; use physical cutoff if movement persists |
| Full run stuck in VERIFY_PICK | Inspect held object and confirm within 120 s, or stop; confirmation is deliberate |
| Cannot start from ERROR/STOPPED | Correct cause, restart only mission terminal C, wait for IDLE, then start |

## 15. Local checks and references

To repeat the offline control checks from the extracted source folder, with ordinary Python 3:

```bash
cd "$JETROVER_REPO"
python3 -B -m unittest discover -s jetrover_retrieve/test -v
```

These cover return convergence in an ideal numerical model, motion limits, test-mode arm isolation, stale sensor stops, invalid depth, too-close handling, operator cancellation, pickup confirmation, and arm-step cancellation. ROS discovery, message transport, camera processing, the driver watchdog, and physical motion require the on-robot checks above.

The guide is self-contained; these optional online references explain the CLI/build commands: [colcon quick start](https://colcon.readthedocs.io/en/released/user/quick-start.html), [ROS 2 Humble topic echo implementation](https://github.com/ros2/ros2cli/blob/humble/ros2topic/ros2topic/verb/echo.py). Project-specific instructions come from the source included in this kit.
