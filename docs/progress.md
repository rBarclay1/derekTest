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

1. Record the current joint positions.
2. Check joints 3 and 4 individually and record useful poses.
3. Check camera depth against measured object distance.
4. Test pickup and placement while stationary.
5. Test approach-only, then the full retrieval sequence.

Arm poses, pickup distance, and grasp success remain unverified. Earlier
battery/IMU telemetry problems have not been conclusively resolved. The
one-command return wrapper has not yet been confirmed on the robot.
