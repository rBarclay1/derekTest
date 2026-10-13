# Progress

## October 13, 2026

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

## Next session

1. Check the arm position and try a small gripper movement.
2. Move one arm joint at a time and record useful poses.
3. Check camera depth against measured object distance.
4. Test pickup and placement while stationary.
5. Test approach-only, then the full retrieval sequence.

Arm poses, pickup distance, and grasp success remain unverified. Earlier
battery/IMU telemetry problems have not been conclusively resolved. The
one-command return wrapper has not yet been confirmed on the robot.
