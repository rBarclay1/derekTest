# Running the JetRover

Robot: Hiwonder JetRover mecanum, Dabai camera, ROS 2 Humble.
Connect the Mac to the robot's Wi-Fi. Use the existing robot shell for all
source commands below.

## Stationary cube search (installed camera-check scripts)

With NetBird connected and the robot application running, SSH to Derek
using the [startup guide](startup-netbird.md). Clear space for the arm,
place the red cube on the floor ahead, and run:

```bash
python3 ~/camera-check/run_camera_search.py
```

The script tilts joint 4 up/down within pulses 149–300, confirms the red
target over several fresh frames, then centers it vertically and exits.
It sends no wheel commands and does not rotate the arm base. Horizontal
centering is not performed. Press Ctrl+C to stop issuing new targets;
the most recent short servo movement can finish.

It stops after 60 seconds if unsuccessful, or on stale camera frames,
missing battery telemetry, or battery below the trial's 10 V cutoff.
Its 20-pixel contour threshold is experimental: it detects red regions,
not cube shape, and far/small targets can still be missed. The launcher
temporarily restores local access to the old ROS hotspot address and
removes that alias when done. It shares a lock with the approach trial
so both scripts cannot run together.

Last image/log on Derek: `~/camera-check/camera-search.png` and
`~/camera-check/camera-search.json`.

## Copy and build

Skip this section if the current code is already installed. Stop the mission
and driver launches with Ctrl+C before replacing files.

**Mac — Terminal 1:**

```bash
cd /Users/Riley/derekTest
scp -r jetrover_retrieve robot_drivers ubuntu@192.168.149.1:~/
scp scripts/run_return_demo.sh jetrover_retrieve/config/longer_return.yaml ubuntu@192.168.149.1:~/
ssh ubuntu@192.168.149.1
```

**Robot — Terminal 1:** back up the installed files, copy the updates, and build.
This uses the robot's existing workspace and dependencies.

```bash
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
sudo systemctl stop start_app_node.service button_scan.service
ros2 node list
sudo fuser -v /dev/ttyACM0
```

If a controller or mission is still running, stop its launch before continuing.
The serial port should have no owner while the drivers are stopped.

```bash
robot_backup="$HOME/jetrover-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$robot_backup"
cp -a /home/ubuntu/ros2_ws/src/jetrover_retrieve "$robot_backup/"
cp /home/ubuntu/ros2_ws/src/driver/controller/controller/odom_publisher_node.py "$robot_backup/"
cp /home/ubuntu/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py "$robot_backup/"
printf '%s\n' "$robot_backup"
cp -R ~/jetrover_retrieve/. /home/ubuntu/ros2_ws/src/jetrover_retrieve/
cp ~/robot_drivers/odom_publisher_node.py /home/ubuntu/ros2_ws/src/driver/controller/controller/odom_publisher_node.py
cp ~/robot_drivers/servo_controller.launch.py /home/ubuntu/ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py
cd /home/ubuntu/ros2_ws
colcon build --symlink-install --packages-select controller servo_controller jetrover_retrieve
source /home/ubuntu/ros2_ws/install/local_setup.zsh
```

Keep the printed build result. Only continue after a successful build.

## Run a mission

Calibrate the arm and pickup distance first. Use a small object and a clear
floor area. Keep the power cutoff within reach. Stop the driving demo and
other controller launches before starting this combined launch.

**Robot — Terminal 1:**

```bash
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
export MACHINE_TYPE=JetRover_Mecanum
sudo systemctl stop start_app_node.service button_scan.service
ros2 launch jetrover_retrieve retrieve_object.launch.py
```

This starts the camera, controllers, and mission. Wait for `IDLE`.
If you already have camera and controller launches running, add
`start_bringup:=false` to use them.

**Mac — Terminal 2:**

```bash
ssh ubuntu@192.168.149.1
```

**Robot — Terminal 2:**

```bash
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
ros2 service call /retrieve_object/start std_srvs/srv/Trigger '{}'
```

Starting saves the current position and heading as home. Watch the output in
Terminal 1. At `VERIFY_PICK`, check that the object is held before running:

```bash
ros2 service call /retrieve_object/confirm_pick std_srvs/srv/Trigger '{}'
```

The robot then returns and places the object in front of its starting
position. To stop from Terminal 2:

```bash
ros2 service call /retrieve_object/stop std_srvs/srv/Trigger '{}'
```

Ctrl+C in Terminal 1 ends the launch. A sent arm move may finish even after
stop; stop does not open the gripper. Restart the mission after `ERROR` or
`STOPPED`. If software cannot stop movement, use the power cutoff and support
anything the arm is holding.

## Settings and camera checks

Edit the source configuration with the mission stopped:

```bash
nano /home/ubuntu/ros2_ws/src/jetrover_retrieve/config/retrieve.yaml
```

Use that file explicitly on the next launch:

```bash
ros2 launch jetrover_retrieve retrieve_object.launch.py config_file:=/home/ubuntu/ros2_ws/src/jetrover_retrieve/config/retrieve.yaml
```

The most useful settings are `target_color`, `max_linear_speed`,
`return_linear_speed`, `max_yaw_speed`, `grasp_distance_min`,
`grasp_distance_max`, and the arm poses. `return_linear_speed` must not exceed
`max_linear_speed`; `search_yaw_speed` must not exceed `max_yaw_speed`.
The launch arguments for color, mode, debug image, auto-start, and pickup
confirmation override YAML values. For example, use `target_color:=blue`
on the launch command to select blue.

Camera topics are `/depth_cam/rgb/image_raw` and `/depth_cam/depth/image_raw`.
Depth must align with the RGB image. Check reported distance against a tape
measure and make sure it follows the object rather than the background.
The initial pickup range is 0.16–0.28 m; it still needs checking against both
camera depth and arm reach.

With the camera already running, the detector can be checked in Terminal 2:

```bash
ros2 launch jetrover_retrieve detect_only.launch.py show_debug_image:=false
```

Its `/detected_object_point` topic reports pixel x/y and depth in meters.
Use `show_debug_image:=true` only with a working graphical display.
Ctrl+C stops the detector. For an approach test, use the mission launch with
`mission_mode:=approach_only`, then call start. It stops in pickup range and
does not command the arm.

## Arm checks

Stop the mission and any old launches. With the serial port free, start only
the hardware driver in **Robot — Terminal 1**:

```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

In sourced **Robot — Terminal 2**, read the actual servo positions:

```bash
timeout 20s ros2 service call /ros_robot_controller/bus_servo/get_state ros_robot_controller_msgs/srv/GetBusServoState "{cmd: [{id: 1, get_position: 1}, {id: 2, get_position: 1}, {id: 3, get_position: 1}, {id: 4, get_position: 1}, {id: 5, get_position: 1}, {id: 10, get_position: 1}]}"
```

Entries are returned in the requested order. Empty positions or a timeout
mean readback has failed, even if `success` is true. The controller's regular
joint-state topic contains cached commands, not measured positions.

Start with small individual movements based on fresh readings and the
physical arm position. IDs 1–5 are arm joints and 10 is the gripper. The
`arm_check` open/close commands go straight to configured targets; they are
not small jogs.

Once individual movements are checked, record `ready_pose`, `grasp_pose`,
`carry_pose`, and `place_pose` in `retrieve.yaml`. Each list alternates servo
ID and pulse, such as `[1, 500, 2, 750, ...]`. Gripper settings are separate.
Numerical pulse limits do not prevent the arm hitting the chassis or floor.

For a calibrated pose check, stop the hardware-only launch in Terminal 1,
then start the controllers there:

```bash
ros2 launch controller controller.launch.py
```

In Terminal 2, test one calibrated pose:

```bash
ros2 run jetrover_retrieve arm_check --operation ready_pose --execute --ros-args --params-file /home/ubuntu/ros2_ws/src/jetrover_retrieve/config/retrieve.yaml
```

Other operations are `grasp_pose`, `carry_pose`, `place_pose`, `open`, `close`,
`ready`, `pick`, and `place`. Pose-only checks leave the gripper alone. Test
empty poses before trying pickup. Completion means the commands were sent;
watch whether the arm reached the intended position.

Ctrl+C stops the arm tool. Its `/arm_check/stop` Trigger service cancels
remaining steps if called from another sourced terminal. A move already
sent can finish.

## One-command driving test

Stop previous launches first. The files copied during setup run a 1 m
outward/return test at up to 0.08 m/s, with no arm commands.

**Robot — Terminal 1:**

```bash
source ~/run_return_demo.sh
```

It starts automatically when ready. Keep SSH connected; Ctrl+C ends the
launch. Edit the settings with the launch stopped:

```bash
nano ~/longer_return.yaml
```

Change `return_test_distance_m`, `return_linear_speed`, and `max_yaw_speed`.
Keep `max_home_distance_m` greater than the test distance and allow enough
time for distance divided by speed in `return_test_timeout_sec`.

## Common problems

| Problem | What to check |
| --- | --- |
| Package or command missing | Source `.robotrc` and `install/local_setup.zsh`; check the build |
| Serial port busy | Stop old launches and both vendor services; inspect `sudo fuser -v /dev/ttyACM0` |
| Controller ready but no movement | Readiness only checks the ROS node; check hardware readback and driver output |
| Stale sensor error | Check camera/odometry messages and their timestamps |
| Missed pickup | Recheck depth, reach, arm poses, and gripper targets |
| Inaccurate return | Measure physical error; odometry can drift when wheels slip |

When finished, stop the mission, then the drivers. Close test publishers
before restarting `start_app_node.service` and `button_scan.service` if you
want to return to vendor operation.
