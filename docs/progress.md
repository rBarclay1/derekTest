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

- Higher-ground-pose from-air trial completed at 19:59 UTC after the user
  explicitly requested removal of the repeat trial's 10 V cutoff. Battery
  telemetry freshness checks remain; voltage was logged (9.852 V start,
  9.789 V finish). Joint 2 target 112 read 102 after lowering and stayed
  102 through release and regrasp, compared with the earlier 92-to-86 drop.
  Gripper opened from 483 to target 283 (measured 284), closed to target
  515 (measured 487), and joint 2 lifted to target 198 (measured 184).
  Color detection found the cube at every stage; final visible center
  (340,315), with bottom-edge clipping and no valid center depth. Final
  full pose `{1:499, 2:184, 3:396, 4:415, 5:499, 10:487}`. Physical pickup
  success and improved release clearance await the user's observation.
  [Trial JSON](calibration/repeat-pickup-from-air-20261027T195939.json);
  full five-stage capture under `repeat-pickup/20261027T195939-6f5a9757/`
  on Derek and Mac. No wheel commands were sent.
- From-air retry prepared to lower joint 2 to 112, open the gripper by
  200, close to 515, then lift joint 2 to 198. Trial stopped before any
  movement because battery read 9.921 V, below its 10 V cutoff.
  Start readings were `{1:499, 2:184, 3:396, 4:415, 5:498, 10:484}`;
  red detected at (346,308). The 112 ground-height adjustment remains
  untested. Initial image/depth and stopped-trial metadata saved under
  `repeat-pickup/20261027T195855-f54b1570/` on robot and Mac; report:
  [from-air retry](calibration/repeat-pickup-from-air-20261027T195855.json).
- User confirmed the recorded repeat trial picked the cube up again.
  On opening, joint 2 appeared to drop lower or be caught on/supported by
  the cube. Readback changed from 92 at ground-closed to 86 at open/closed.
  Contact/load interaction is a hypothesis, not a confirmed cause.
  Next proposed ground-level joint 2 target: **112 instead of 92**,
  a 20-pulse increase to test additional clearance before opening. This
  adjusted pickup height has not yet been tested; verify actual gripper
  clearance as well as servo readback. No new arm motion was commanded
  when recording this observation.
- Recorded release/regrasp/lift repeat on 2026-10-27 at 19:56 UTC:
  [trial JSON](calibration/repeat-pickup-20261027T195623.json).
  The stationary sequence opened ID 10 from measured 484 to target 284
  (measured 286), closed to target 515 (measured 481), then lifted joint 2
  to target 198 (measured 184). Other joints were not commanded.
  Final full readback: `{1:499, 2:184, 3:396, 4:415, 5:498, 10:483}`.
  Color detection found red at every recorded stage: ground closed
  (341,338), open (348,305), closed (348,309), lifted (346,308).
  Nearly constant image location during lift is consistent with carrying,
  but does not independently prove retention. The cube touches the bottom
  image edge and its sampled depth is invalid in all four snapshots.
  User subsequently confirmed this repeat trial lifted the cube. Battery was
  10.162 V initially and 10.098 V after lifting. Joint 2 drifted to 86
  during the open/close stages despite its earlier target 92; commanded
  pulses and measured positions must remain distinct in calibration.
  Four RGB/depth/metadata snapshots saved under
  `~/grip-data/repeat-pickup/20261027T195623-5d092dee/` on Derek and
  `/Users/Riley/jetrover-grip-data/repeat-pickup/20261027T195623-5d092dee/`
  on the Mac. Script: `scripts/repeat_pickup_trial.py`; requires this
  calibrated ground pose and does not search, approach, or drive.
- Successful lift and held reference saved 2026-10-27 at 19:54:01 UTC:
  user confirmed the cube is lifted and held securely. Measured positions:
  `{1: 499, 2: 183, 3: 396, 4: 414, 5: 498, 10: 483}`.
  Relevant last commanded targets: joint 2: 198, joint 3: 400,
  joint 4: 417, gripper: 515. The stronger grip followed the failed lift
  below. Visible red-region center is (341, 340); the cube is clipped at
  the bottom edge and its center depth is invalid. Record:
  [held-state.json](calibration/held-state.json). RGB, depth, annotation,
  and full metadata saved in `cube-grip-calibration/20261027T195401-held-ea79bed6/`
  under `~/grip-data/` on Derek and `/Users/Riley/jetrover-grip-data/`
  on the Mac. This records one successful manual pickup, not a validated
  autonomous sequence. No movement commanded while saving this state.
- Lift test after saving the reference failed: user reported the grip was
  insufficient when joint 2 moved from 92 toward 200 (measured 187), with
  gripper measured 467. The saved 469 closing target is a contact reference,
  not a validated carrying grip. Joint 2 was returned to target 92,
  measured 93; gripper then opened by 50 to target 417, measured 420.
  Cube detected afterward at (337, 341). The saved reference JSON now
  explicitly records the failed lift; its original joint/image data remain.
- Fully gripped reference, refreshed 2026-10-27 at 19:48:19 UTC:
  measured joint positions `{1: 499, 2: 92, 3: 396, 4: 415, 5: 501, 10: 467}`.
  Gripper closing command was 469. Color detection on the saved RGB frame
  found the visible red contour center at **(347, 336)** in a 640 × 360
  image; bounding box `(x=278, y=312, width=144, height=48)`.
  The cube is clipped at the bottom edge, so this is the visible-region
  centroid, not the full cube center. Depth at that location was invalid.
  User confirmed the grip; lifting remains unverified. This supersedes the
  earlier sample as the current fully gripped reference.
  Machine-readable record: [fully-gripped-state.json](calibration/fully-gripped-state.json).
  RGB/depth, detection image, and full metadata are saved under
  `~/grip-data/cube-grip-calibration/20261027T194819-grasp-ded27e76/` on Derek
  and `/Users/Riley/jetrover-grip-data/cube-grip-calibration/20261027T194819-grasp-ded27e76/`
  on the Mac.
- 2026-10-27 confirmed red-cube grip: user reported fully gripping the
  cube after commanding gripper ID 10 to 469; measured position 465.
  Saved closing target: 469. This confirms the stationary grip, not a
  successful lift or autonomous pickup. Current measured pose at capture:
  ID 1: 499, ID 2: 92, ID 3: 396, ID 4: 415, ID 5: 501, ID 10: 465.
  RGB, depth, camera calibration, and servo readings saved on Derek under
  `~/grip-data/cube-grip-calibration/20261027T194634-grasp-160b0fd2/`
  and on the Mac under
  `/Users/Riley/jetrover-grip-data/cube-grip-calibration/20261027T194634-grasp-160b0fd2/`.
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

- Stationary search-and-center script added and live-tested. From joint 4
  measured 149, it scanned upward, detected the cube entering the top of
  the image near (321, 3), and centered it at (321, 175). Joint 4 final
  readback was 269, last target 272. Only joint 4 was commanded; no wheels.
  Five hardware-independent tests cover scan reversal, pulse limits,
  repeated-detection confirmation, target loss, and centering direction.
  Run on Derek: `python3 ~/camera-check/run_camera_search.py`.
  It scans 149–300, uses minimum red contour area 20, stops after vertical
  centering, and times out after 60 seconds. Far-cube and other-red-object
  limitations remain. Results saved as `camera-search.png/json`.
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
