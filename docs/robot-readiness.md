# Deferred robot work

Implementation was prepared without the robot. Fifteen offline Python control
checks now pass with mocked ROS interfaces; no ROS build or robot motion has
been run. Hardware readiness is still unverified.

Use the [offline first-session guide](offline-robot-guide.md) for exact commands,
expected outcomes, calibration, and recovery. A standalone browser/print version
is included as [offline-robot-guide.html](offline-robot-guide.html).

The confirmed robot is `JetRover_Mecanum`. The first-session overrides select
this chassis and lower speeds. New `approach_only` and `return_test` mission
modes isolate base trials from arm movement; `arm_check` supports stationary
arm calibration and cancellation. The detector now accepts the same custom
configuration file as the mission.

## What is implemented

- One mission with explicit start/stop, saved home pose, color tracking,
  depth-based approach, pickup, return, placement, and terminal status.
- Freshness checks for RGB/depth/odometry and timestamp skew checks for depth
  sampling; invalid depth never authorizes forward motion.
- Configurable mission, approach, search, return, arm, and confirmation
  timeouts; maximum displacement from home.
- Arm actions that keep callbacks responsive and cancel future steps on stop.
- Consistent gripper pulse bounds for actions, poses, and manual dashboard.
- Driver command expiry independent of the mission process.
- Manual grasp confirmation by default and an explicit unverified automatic
  mode. Completing a timed servo sequence does not prove physical success.

## First hardware session, in order

1. Preserve the installed controller source and deploy the mission package
   plus the modified controller file. Confirm the chassis model and robot
   environment. The backup omits packages installed on the real robot.
2. Inspect actual camera/depth/odometry topic names, timestamp behavior, QoS,
   and rates. Check that `/odom` has stable parent/child frame IDs and follows
   the physical base rather than a camera frame.
3. Confirm RGB/depth registration using a known object. Check that the depth
   at its RGB center is its measured distance. Verify invalid/no-return
   pixels, units, and operation at 0.16–0.28 m; the camera may have a larger
   minimum usable range than the configured pickup range.
4. Confirm command expiry, mission stop, camera/depth/odometry loss handling,
   and timeouts before any full mission. Check that a stop during an arm
   sequence suppresses subsequent steps while the current move may finish.
5. Calibrate gripper open/closed pulses, ready/grasp/carry/place poses, action
   durations, and pickup range with stationary objects. Confirm the carrying
   pose holds the object and the placement pose reaches the intended surface.
6. Use detector-only operation to tune HSV ranges and object area. Check
   multiple same-color objects and lighting changes before enabling approach.
7. Check alignment direction and slow approach with arm action disabled or
   supervised. Verify the configured too-close abort and lost-target stop.
8. Check short return trips without a carried object using a controlled
   mission setup. Compare the reported home tolerance with physical error;
   tune odometry and consider a home marker before increasing range.
9. Run one supervised mission with manual pickup confirmation. Inspect the
   physical drop location and status transitions.
10. Enable unverified automatic return only if that behavior is acceptable.
    Otherwise implement grasp feedback before making the mission fully
    autonomous. Add obstacle-aware navigation before using cluttered spaces.

## Future implementation work

- Add a measured pickup-success signal and bounded recovery attempts. Do not
  treat servo position alone as evidence that an object is held.
- Add laser/depth obstacle checks and a navigation planner for return routes.
- Add a home marker or map localization for final placement accuracy.
- Transform camera observations into the arm/base coordinate system using
  calibrated camera intrinsics/extrinsics if pickup must work at varied
  locations, heights, or object sizes.
- Add object identity tracking beyond nearest same-color contours.
- Add ROS integration checks and sensor/driver fault testing on the robot.
  Existing offline checks cover control logic; they cannot validate timing,
  calibration, physical grasping, or command expiry in the installed driver.
