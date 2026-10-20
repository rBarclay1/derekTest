# Camera data during grip calibration

Use `scripts/capture_grip.py` to save a stationary camera sample and measured
servo positions. It subscribes to the camera and requests servo readback;
it sends no motion commands. The script runs directly, so no rebuild is needed.

## Copy once

**Mac — Terminal 3**, connected to the robot's Wi-Fi:

```bash
scp /Users/Riley/derekTest/scripts/capture_grip.py ubuntu@192.168.149.1:~/
ssh ubuntu@192.168.149.1
```

## Start the camera alongside the arm driver

**Robot — Terminal 1:** keep the existing hardware driver running. Stop any
retrieval mission or driving demo before calibration. If no hardware driver is
running, follow the arm startup steps in the [robot guide](offline-robot-guide.md).

**Mac — Terminal 2:** open another SSH connection if needed:

```bash
ssh ubuntu@192.168.149.1
```

**Robot — Terminal 2:**

```bash
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
ros2 node list
```

If the depth camera is already running, keep its existing launch. Otherwise
start it here and leave this terminal running:

```bash
ros2 launch peripherals depth_camera.launch.py
```

This camera launch does not start another arm driver. Let the camera run for
about five seconds so its exposure can settle.

**Robot — Terminal 3:**

```bash
source /home/ubuntu/ros2_ws/.robotrc
source /home/ubuntu/ros2_ws/install/local_setup.zsh
```

This terminal is available for manual arm commands and captures between moves.

## Collect one trial

Use a matte object, steady lighting, and the same floor and camera mounting
throughout the first trial. Put a ruler or tape mark beside the pickup spot.
Record the object's size and approximate distance in `--notes`. Keep fingers
out of the camera view and wait until each arm move has finished before capturing.
Hold the robot and object still until the script exits: servo readings are
collected just after the images, not at exactly the same instant.

Start with the arm in Ready and the object on the floor. In **Robot — Terminal 3**:

```bash
python3 ~/capture_grip.py --trial trial01 --stage ready --object red-block --notes "Object on floor; arm Ready"
```

Change `red-block` to describe your object. This first sample shows where the
object appears before pickup. Review it before continuing.

After manually positioning the fingers around the object and closing them,
with the object still on the floor:

```bash
python3 ~/capture_grip.py --trial trial01 --stage grasp --object red-block --notes "Fingers closed; object still on floor"
```

After lifting, only use `held` if you can see that the object is securely held:

```bash
python3 ~/capture_grip.py --trial trial01 --stage held --object red-block --notes "Object lifted and held in carry pose"
```

For comparison, capture the same carry pose with an empty gripper. Use the
same closing target when practical, and record any difference:

```bash
python3 ~/capture_grip.py --trial trial01 --stage empty --object red-block --notes "Empty gripper in carry pose; object removed from view"
```

Each command creates a new folder, so repeating a capture does not overwrite
an earlier one. Take three samples at each stage, then repeat the pickup as
`trial02` and `trial03`. Also record missed grips and dropped objects with an
accurate stage and note. These examples help us distinguish a held object from
a visible object that is still on the floor.

## What gets saved

Files go under `~/grip-data/<trial>/<timestamp>-<stage>-<unique ID>/` on the robot.

| File | Contents |
| --- | --- |
| `rgb.png` | Full-resolution color image, without markings. |
| `depth_raw.npy` | Original depth array, preserving the camera's values and units. |
| `depth_meters.npy` | Depth in meters. Missing, zero, or invalid distances become NaN. |
| `depth_preview.png` | Viewable depth image: near is bright, far is dark, missing is black. Display range is 0.05–2 m. |
| `metadata.json` | Stage, notes, object name, image timestamps, encodings, frame IDs, camera calibration if available, and measured servo positions. |

Depth files are saved only when a recent matching depth frame can be decoded.
The script waits up to ten seconds for camera data. It rejects RGB/depth pairs
more than 0.10 seconds apart and frames more than one second old. With no usable
depth pair, it saves a fresh RGB image alone and records the issue.

A depth frame can still have holes around a close or reflective object. The
reported valid-depth fraction covers the whole image, not just the object.
Do not interpret it as a successful grip or a valid object distance.

The vendor launch requests depth registration, but that still needs checking:
compare the object's outline in RGB and depth at several floor positions.
Matching image sizes alone do not prove alignment. Metadata leaves
`registration_verified` false until we establish this separately.

Missing camera calibration or servo readings are recorded explicitly. If a
servo request times out, the driver may be blocked waiting for hardware; stop
and inspect Terminal 1 before sending another read or movement. For camera-only
captures while diagnosing readback, add `--skip-servos`.

## Copy and inspect on the Mac

In a **Mac terminal** while still connected to the robot's Wi-Fi:

```bash
mkdir -p /Users/Riley/jetrover-grip-data
scp -r ubuntu@192.168.149.1:~/grip-data/. /Users/Riley/jetrover-grip-data/
open /Users/Riley/jetrover-grip-data
```

Open `rgb.png` and `depth_preview.png` from the first Ready sample. Check that
the image is sharp, the object is visible, and the floor is not washed out.
Compare the held and empty samples to see whether the camera can actually see
the object while carried. If fingers block it, note that rather than treating
an empty image region as proof the pickup failed.

The `.npy` files are for later analysis with NumPy. Keep the image folders and
metadata together. Captures are stored outside the code repo.
