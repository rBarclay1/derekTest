# Driver changes

These two files are based on the Hiwonder drivers from the robot workspace.
They are replacements for individual files, not standalone ROS packages.
The robot still needs its installed vendor packages, including the camera,
robot description, controller, and servo message packages.

| File | Location under `/home/ubuntu/ros2_ws/src/` | Change |
| --- | --- | --- |
| `odom_publisher_node.py` | `driver/controller/controller/odom_publisher_node.py` | Stop wheel commands after 0.5 seconds without a new velocity command; reject non-finite velocities and send zero on shutdown |
| `servo_controller.launch.py` | `driver/servo_controller/launch/servo_controller.launch.py` | Import the package lookup used by compiled installs |

The odometry driver estimates travel from commanded speeds. Its position
estimate can differ from actual travel, especially when the wheels slip.

Installation and backup commands are in the [robot guide](../docs/offline-robot-guide.md).
