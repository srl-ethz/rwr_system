# `hand_control` package
Joint-level controllers for the hand
see Notion page [Hand Control Hello World](https://www.notion.so/Hand-control-hello-world-223bcc457b5880a4ad73e1b2e3dad025) for more detailed usage and tutorial

## to test connection to individual Dynamixels

```bash
# connect to motors 0, 1, 2 on port /dev/ttyUSB0
python motor_control_gui.py -m 0,1,2 -p /dev/ttyUSB0
# there are other options, see help
python motor_control_gui.py -h
```

## ORCA Hand Joint GUI
```bash
cd orca
python joint_gui.py
```
Calibration can be run from the GUI. After initializing, each joint can be controlled manually with the sliders.

## ORCA Hand base ROS controller

```bash
ros2 launch hand_control orcahand_v1b_controller_launch.yaml hand_chirality:=right calibrate:=true launch_rviz:=true compliant_test_mode:=true 
```
in `compliant_test_mode`, the joints become limp and the proprioception shows up in Rviz.


## ORCA_v2 Hand 
For standalone v2 chirp acquisition and offline validation (without ROS), see
the [ORCA characterization guide](../../experiments/orca/README.md).

The v2 hand is driven through the [orca_core](https://pypi.org/project/orca_core/) package. Install it once into the ROS environment. `--no-deps` is needed because orca_core asks for a newer numpy version than faive pins; it works with faive's numpy version.
```bash
pip install --no-deps orca_core==0.5.2
```

**1. Tension the tendons** (when needed, e.g. after assembly or when tendons are slack). Press Ctrl+C when done:
```bash
cd orca
python tensioning_v2.py
```

**2. Run the hand:** Pick ONE of the following, they cannot run together:

- **(a) Controller only:** for policies, teleop or your own nodes that publish to `/right/hand/policy_output`:
  ```bash
  ros2 launch hand_control orcahand_v2_controller_launch.yaml hand_chirality:=right launch_rviz:=true
  ```
- **(b) Joint GUI:** move each joint with sliders. This starts its own controller and RViz, so do not run (a) at the same time:
  ```bash
  ros2 launch experiments orcahand_v2_joint_gui.launch.py
  ```

**3. Calibration:** Without a stored calibration the hand calibrates automatically when (a) or (b) starts. After tensioning, recalibrate once by starting (a) with `calibrate:=true` (keep your hands clear while the joints move to their end stops). The calibration is saved as `calibration.yaml` next to the installed config (`install/hand_control/share/hand_control/orca/configs/orcahand_v2/right/`), so deleting `install/` means calibrating again.

