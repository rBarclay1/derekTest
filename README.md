# JetRover Object Retrieval

A ROS 2 project for the Hiwonder JetRover with a mecanum chassis and Dabai
camera. The robot looks for a colored object, drives toward it, picks it up,
returns to its starting position, and puts it down in front of the robot.

## Progress

The package and driver changes have been built on the robot. A driving test
went about 1 meter forward at 0.08 m/s, returned, and restored the starting
heading. Position readings came back from all five arm servos and the gripper.
Arm movement, pickup, and the full retrieval sequence still need testing.
The robot lost power before the first gripper movement was confirmed.

See [progress and arm readings](docs/progress.md) for the current notes.

## Setup and use

After a restart, follow the [startup and NetBird guide](docs/startup-netbird.md)
to connect Wi-Fi, fix the clock, and SSH from the Mac.

Follow the [robot guide](docs/offline-robot-guide.md) for copying, building,
launching, stopping, and calibrating. It has separate Mac and robot commands.
The robot uses its existing ROS 2 Humble installation and Hiwonder drivers.

Edit `jetrover_retrieve/config/retrieve.yaml` for the target color, speeds,
pickup distance, and arm poses. The supplied arm poses are starting values;
calibrate them before running a full mission.

The mission follows these steps:

```text
prepare arm → search → approach → pick → confirm pickup → return → place
```

Starting a mission saves the current position and heading as home. After
pickup, the robot waits for confirmation that it is holding the object.
Return uses odometry and a direct route, so it needs a clear floor area.
There is no obstacle avoidance, and wheel slip can affect the return position.

Two test modes help check parts of the mission:

- `approach_only`: find and approach the object without moving the arm.
- `return_test`: drive out and return without moving the arm.

## Files

See the [file structure guide](docs/file-structure.md) for the full folder tree
and an explanation of every file.

| File or folder | What it does |
| --- | --- |
| `jetrover_retrieve/jetrover_retrieve/retrieve_object_node.py` | Main mission loop and start/stop services |
| `jetrover_retrieve/jetrover_retrieve/color_detect.py` | Color detection and depth measurement |
| `jetrover_retrieve/jetrover_retrieve/navigation.py` | Calculate the drive back to home |
| `jetrover_retrieve/jetrover_retrieve/arm_control.py` | Arm poses and pickup/place steps |
| `jetrover_retrieve/jetrover_retrieve/arm_check.py` | Test one arm operation |
| `jetrover_retrieve/config/` | Mission and driving-demo settings |
| `scripts/run_return_demo.sh` | Start the driving demo with one command |
| `robot_drivers/` | Two modified Hiwonder driver files |
| `docs/status-reports/` | Class progress reports |

The driver files come from the Hiwonder robot software. See
[driver notes](robot_drivers/README.md) for the changes and install locations.

## Local tests

From the repository folder:

```bash
python3 -B -m unittest discover -s jetrover_retrieve/test -v
```

These check the control logic with ROS interfaces mocked. They do not test
ROS communication or physical movement.
