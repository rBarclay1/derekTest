# Repository file guide

Most of the project code is in `jetrover_retrieve/`. The other folders hold
instructions, a driving script, and two changes to the robot's existing drivers.

The labels in the tree show where each file came from:

- **[P] Project:** made for this project. The ROS package and some scripts have
  also been copied onto the robot; that does not make them original robot files.
- **[M] Modified robot file:** started as a Hiwonder file from the robot and
  was changed for this project. The copy here can replace that file on the robot.
- **[R] Original robot file:** copied from the robot without changes. There are
  no [R] files in the current repo; the old bulk robot backup was removed.

These labels describe the files' origin. They do not tell us whether the copy
currently installed on the robot matches this repo.

```text
derekTest/
├── README.md [P]
├── .gitignore [P]
├── docs/
│   ├── file-structure.md [P]
│   ├── offline-robot-guide.md [P]
│   ├── progress.md [P]
│   └── status-reports/
│       └── week-8/
│           ├── Status Report - Week 8 Update.pdf [P]
│           └── Status Report - Week 8 Update.pptx [P]
├── jetrover_retrieve/
│   ├── package.xml [P]
│   ├── setup.py [P]
│   ├── setup.cfg [P]
│   ├── resource/
│   │   └── jetrover_retrieve [P]
│   ├── config/
│   │   ├── retrieve.yaml [P]
│   │   └── longer_return.yaml [P]
│   ├── launch/
│   │   ├── retrieve_object.launch.py [P]
│   │   └── detect_only.launch.py [P]
│   ├── jetrover_retrieve/
│   │   ├── __init__.py [P]
│   │   ├── retrieve_object_node.py [P]
│   │   ├── color_detect.py [P]
│   │   ├── navigation.py [P]
│   │   ├── arm_control.py [P]
│   │   ├── arm_check.py [P]
│   │   └── object_detector_node.py [P]
│   └── test/
│       └── test_retrieval.py [P]
├── robot_drivers/
│   ├── README.md [P]
│   ├── odom_publisher_node.py [M]
│   └── servo_controller.launch.py [M]
└── scripts/
    └── run_return_demo.sh [P]
```

## Root files

| File | Purpose |
| --- | --- |
| `README.md` | Project overview, current progress, and links to setup instructions. Start here. |
| `.gitignore` | Keeps generated files, build output, and Python caches out of Git. |

Git also keeps a hidden `.git/` folder for commits and repository settings.
It is managed by Git and is not part of the robot code.

## `docs/` — instructions and progress

| File or folder | Purpose |
| --- | --- |
| `file-structure.md` | This guide. |
| `offline-robot-guide.md` | Mac and robot commands for installation, running, stopping, calibration, and troubleshooting. |
| `progress.md` | Completed hardware checks, recorded servo readings, and next steps. |
| `status-reports/` | Class progress reports, grouped by week. |
| `status-reports/week-8/Status Report - Week 8 Update.pptx` | Editable Week 8 presentation. |
| `status-reports/week-8/Status Report - Week 8 Update.pdf` | PDF version of the Week 8 report. |

## `jetrover_retrieve/` — the ROS package

This outer folder is copied into `/home/ubuntu/ros2_ws/src/` on the robot.
It contains the Python code plus the files ROS needs to build and run it.

| File or folder | Purpose |
| --- | --- |
| `package.xml` | Package name, description, and required ROS and Python dependencies. |
| `setup.py` | Installs the Python code, launch files, and settings. Registers the `retrieve_object`, `object_detector`, and `arm_check` commands. |
| `setup.cfg` | Sets the install location for those commands so `ros2 run` can find them. |
| `resource/jetrover_retrieve` | Empty marker file installed into the ROS package index so ROS can find this package. |

### `config/` — editable settings

| File | Purpose |
| --- | --- |
| `retrieve.yaml` | Main mission settings: target color, camera topics, speeds, timeouts, pickup distance, arm poses, and gripper targets. |
| `longer_return.yaml` | Settings for the 1-meter driving demo, including speed and return limits. |

Change these files when tuning the robot. The launch files can override some
settings, such as target color and mission mode. Arm poses still need calibration.

### `launch/` — starting ROS programs

| File | Purpose |
| --- | --- |
| `retrieve_object.launch.py` | Starts the mission with its settings. Also starts the camera and controllers unless `start_bringup:=false` is used. |
| `detect_only.launch.py` | Starts the color/depth detector for camera checks. Requires the camera to already be running. |

### Inner `jetrover_retrieve/` — Python code

The repeated folder name is normal for a Python ROS package: the outer folder
holds the whole ROS package, and the inner folder holds the Python modules.

| File | Purpose |
| --- | --- |
| `__init__.py` | Marks the folder as a Python package so its files can be imported. |
| `retrieve_object_node.py` | Main mission loop. Reads camera images and odometry, saves home, searches, approaches, calls the arm steps, and returns. Handles start, stop, pickup confirmation, and timeouts. |
| `color_detect.py` | Finds red, green, or blue objects using OpenCV, follows a nearby target between frames, and reads depth around its image position. |
| `navigation.py` | Reads position and heading from odometry and calculates forward/turn speeds for returning home. |
| `arm_control.py` | Validates servo targets and sends the timed moves for ready, pick, carry, and place. Uses the poses from the settings. |
| `arm_check.py` | Runs one selected arm operation for calibration. Requires `--execute`; its stop service cancels remaining steps. |
| `object_detector_node.py` | Runs the camera detector separately. Publishes the object's pixel x/y and depth in meters for checking detection. |

The main mission calls `color_detect.py`, `navigation.py`, and `arm_control.py`
directly. It does not need the separate detector node running. `arm_check.py`
also uses `arm_control.py`, so its pose checks use the same arm code as the mission.

### `test/` — offline checks

`test_retrieval.py` checks return navigation, mission transitions, sensor
timeouts, stopping, arm sequence timing, and servo settings. It replaces ROS
interfaces with test objects so it can run on the Mac without a connected robot.

## `robot_drivers/` — changes to Hiwonder drivers

These files replace individual files in the robot's installed driver packages.
This folder does not contain all the drivers needed to run the robot.
The two Python launch/driver files here are [M]; the README in this folder is [P].

| File | Purpose |
| --- | --- |
| `README.md` | Explains the driver changes and where each file goes on the robot. |
| `odom_publisher_node.py` | Converts drive commands into wheel commands and estimates travel. Includes a timeout that stops the wheels if new velocity commands stop arriving. |
| `servo_controller.launch.py` | Starts the installed servo controller. Includes the package lookup import needed when using compiled installs. |

## `scripts/` — driving demo

`run_return_demo.sh` loads the robot environment, stops the competing vendor
services, checks for an existing controller or mission, and launches the return
test. It reads `~/longer_return.yaml` on the robot. After both files have been
copied to the robot's home folder, run it in the existing robot shell with:

```bash
source ~/run_return_demo.sh
```

The test drives out, returns to the saved starting position and heading, and
does not command the arm.
