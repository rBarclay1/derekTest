# Progress



######

2. Robot — Terminal 1: start the hardware driver

Once those checks are clear:

ros2 launch ros_robot_controller ros_robot_controller.launch.py

Leave this running.

3. Mac — Terminal 2: connect

ssh ubuntu@192.168.149.1

Robot — Terminal 2: read the gripper

source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh

4. Only if the fresh reading is between 495 and 505, and the gripper has room to move slightly in either direction, run this once in Robot — Terminal 2:

ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 1.0, position: [{id: 10, position: 509}]}"

######



- Installed and built the retrieval package and modified drivers.
- Drove about 1 meter forward at 0.08 m/s, returned, and restored heading.
- Read positions from all five arm servos and the gripper.
- The robot lost power before a gripper movement was confirmed.

| Servo | Reported pulse |
| --- | ---: |
| 1 | 499 |
| 2 | 764 |
| 3 | 9 |
| 4 | 148 |
| 5 | 500 |
| 10 (gripper) | 499 |

These are readings from that session, not calibrated targets. Read the
positions again after powering up before choosing a small movement.

## Arm movement observations

- 2026-10-27: Joint 2 (shoulder) at about 800 hits the monitor at the
  tested base rotation (ID 1 target 549, measured 546). Avoid this position
  at that base rotation; a safe clearance limit has not been established.
  Joint 2 was subsequently moved down from 807 to target 707, measured 708.
- After restarting, the small gripper movement to a target of 509 worked.
  Measured readback has not been recorded.
- Gripper (ID 10): 200 is fully open and 700 is fully closed.
- Gripper rotation (ID 5): 400 turns left and 600 turns right.
- Base rotation (ID 1): higher pulses turn left/counterclockwise; lower
  pulses turn right/clockwise. The viewing direction has not been recorded.
- Joints 2–4 point straight up at pulse 500, as observed on the robot.
- For joint 2 (shoulder), 400 brings it down and 600 brings it up.
  These directions describe the tested arrangement of the arm.
- Directions for increasing/decreasing pulses on joints 3 and 4 still need
  recording. These observations do not establish safe travel limits.

## Candidate Ready pose

Mostly collapsed, with the camera looking forward. Recorded from the last
published command for each servo in the manual test log, not measured readback.

| Servo ID | Joint | Last commanded pulse |
| --- | --- | ---: |
| 1 | Base rotation | 500 |
| 2 | Shoulder | 750 |
| 3 | Elbow | 0 |
| 4 | Wrist pitch | 325 |
| 5 | Gripper rotation | 500 |
| 10 | Gripper | 200 |



ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 1.0, position: [{id: 10, position: 509}, {id: 10, position: 509}]}"


## GRIPPER CLOSED IS 700 on nothing tho

## need to know what the gripper pos is when on cube

## PICKUP
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 2.0, position: [{id: 10, position: 350}, {id: 4, position: 400}, {id: 3, position: 400}, {id: 2, position: 100}]}"

## READY
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 2.0, position: [{id: 10, position: 200}, {id: 4, position: 325}, {id: 3, position: 0}, {id: 2, position: 750}]}"


## CLOSE AND READY
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 2.0, position: [{id: 3, position: 0}, {id: 2, position: 600}, {id: 4, position: 325}, {id: 10, position: 200}]}"



Arm pose pairs: `[1, 500, 2, 750, 3, 0, 4, 325, 5, 500]`.
The gripper is fully open at 200. This pose is recorded here for calibration;
the mission settings have not been updated. Returning to it as a combined
move has not been tested.

## Next steps

### Camera check — 2026-10-27

- At the user's approximate 3-foot placement, camera pose unchanged,
  the standard HSV/morphology pipeline detected the cube in 12/12 sampled
  frames with minimum contour area 20. Thresholds 200 and 500 each gave
  0/12. Center was about (302, 222–223), filtered area 30–54.5 pixels,
  and depth 1.007–1.018 m. This is a short stationary observation, not
  validation against other red objects or changing lighting. The user's
  distances are approximate ground placements; camera depth is measured
  from a different origin. Saved `three-feet*` artifacts. No movement
  commanded during this comparison.
- At the user's approximate 5-foot placement, with the camera pose held
  fixed after the far-distance test, a seeded measurement found the cube
  at (305, 173), about 8 × 8 pixels and 34–39 pixels of red contour area.
  The standard detector still failed even with minimum area 20: its color
  threshold and erosion reject the small region, so lowering area alone
  is insufficient. A relaxed color mask in a manually selected region
  found it in 12 recent frames. Candidate depth was mostly 1.623 m
  (range 1.597–1.651 m); nearby floor below it read 1.434–1.455 m.
  These are diagnostic measurements, not validated autonomous detection
  or a precisely measured ground-truth distance. No arm/wheel motion was
  commanded for this test. Saved `five-feet*` artifacts.
- Far-distance test: cube placed beyond the initial view. Joint 4 was
  raised incrementally from about 163 to 236; the cube became visible
  near pixel (302, 51). Its colored region was only about 5 × 6 pixels.
  Standard HSV saturation and erosion removed it. Relaxing the filter
  across the full image also detected chair parts, so it is not a reliable
  far-distance detector yet.
- A diagnostic tracker seeded at the visually identified cube location
  followed it while joint 4 increased from 237 to 292. Cube image y moved
  from about 53 to 151. The trial stopped when repeated detections were
  lost; the cube was not fully centered at y=180. Other arm joints and
  wheels were not commanded. This was a seeded calibration test, not
  autonomous cube acquisition. Actual floor distance and valid cube depth
  at this far position have not been measured. Saved `far-*.png/json` in
  `artifacts/camera-check/`.
- After recharging, battery read 10.882 V. Initial servo readings:
  ID 1: 500, ID 2: 764, ID 3: 7, ID 4: 149, ID 5: 501, ID 10: 498.
  The red cube was at pixel (315, 141), with camera depth about 0.420 m.
- Small camera-tilt calibration: joint 4 target 164, measured 162, from
  measured 149. Cube center shifted from (315, 141) to (315, 176).
  Increasing joint 4 moved the cube downward in this tested arrangement,
  about 2.7 pixels per measured pulse unit locally. This slope is a local
  estimate, not a calibration across all arm poses.
- A 22-second stationary vertical-tracking trial used joint 4 limits
  149–180, at most 3 pulse units per correction, an 8-pixel dead zone,
  and fresh, repeated detections. All 743 frames had a red detection.
  The cube stayed at about (315, 176), so no corrective commands were
  needed. Following a moving cube remains unverified. No wheel commands
  were sent. Diagnostic JSON and the marked image are in
  `artifacts/camera-check/`.
- Dabai RGB and depth streams received at about 30 fps, 640 × 360.
  The driver reports hardware depth-to-color alignment; both camera-info
  streams use the color optical frame and matching intrinsics.
- Red cube detected at pixel (327, 194), 7 pixels right of image center.
  Twelve recent frames returned depth readings of 0.513–0.515 m.
  This is camera depth, not distance from the chassis or gripper; compare
  against a physical measurement before approaching.
- The red contour measured about 446–465 pixels. The default 500-pixel
  minimum rejected it. A 200-pixel minimum worked for this stationary test.
- Detection-only launch now accepts `min_object_area:=200.0`. The mission's
  default area threshold remains unchanged. No driving was performed.
- Snapshot and detection details: `artifacts/camera-check/`.

```bash
ros2 launch jetrover_retrieve detect_only.launch.py target_color:=red min_object_area:=200.0
```

1. Record the current joint positions.
2. Check joints 3 and 4 individually and record useful poses.
3. Check camera depth against measured object distance.
4. Test pickup and placement while stationary.
5. Test approach-only, then the full retrieval sequence.

Arm poses, pickup distance, and grasp success remain unverified. Earlier
battery/IMU telemetry problems have not been conclusively resolved. The
one-command return wrapper has not yet been confirmed on the robot.
