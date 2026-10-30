# Retargeter

The retargeter maps the human hand keypoints from the Rokoko glove to the joint angles of the ORCA v2 hand. For each frame it optimizes the joint angles so that the vectors between the robot's fingertips, finger bases and palm match those of the human hand.

`retargeter_node.py` subscribes to `/ingress/<chirality>/mano` (22 keypoints: forearm, then the 21 MANO points starting at the wrist) and publishes the joint angles in radians to `/<chirality>/hand/policy_output`. The service `/<chirality>/retargeter/set_enabled` (`std_srvs/SetBool`) pauses and resumes it. The teleop and recording launch files in `experiments/launch` start it.

## Configuration

| File | Content |
|---|---|
| `src/viz/models/orcahand_v2/scheme_orcahand_v2.yaml` | Joint names, joint limits, and which robot links are the fingertips, finger bases, palm and forearm |
| `experiments/cfgs/retargeter/retargeter_cfgs_orcahand_v2.yaml` | Learning rate and loss weights, e.g. how strongly each joint is pulled towards a default angle |
| `experiments/cfgs/retargeter/mano_adjustment_orcahand_v2.yaml` | Translation, rotation and scale applied to the human keypoints (all of them, then per finger) before retargeting, to compensate for the different hand shapes |

## Tune the retargeter

Start the glove, the hand and the config helper:
```bash
ros2 launch experiments retargeter_config_helper.launch.py
```
The helper reloads the two files in `experiments/cfgs/retargeter/` whenever you save them, so you can edit the adjustments and watch the result in RViz and on the hand. With `colcon build --symlink-install` the installed files link to the source files, so edit those in the repository.

To work without the glove, set `use_saved_keypoint = "hand_open"` in the launch file: the helper then uses a stored open-hand pose.

## Calibrate the MANO adjustments from data

Instead of tuning by hand, fit the global adjustment (`all`) to pairs of glove keypoints and the matching robot joint angles:

1. Start the teleop (e.g. `ros2 launch experiments teleop_rokoko.launch.py`) and, in another terminal, the collector:
    ```bash
    ros2 run retargeter retargeter_collect_calibration_data.py
    ```
2. In the collector's terminal: press Enter to pause the retargeter and make the hand compliant, shape the robot hand by hand to match your own hand pose, press `s` + Enter to save the pair, and Enter again to resume. Repeat for different poses, then `q` + Enter to write the JSON file (next to the installed script).
3. Fit the adjustments:
    ```bash
    python src/retargeter/retargeter_calibration.py -d retargeter_calibration_right_<time>.json
    ```
    It writes `mano_adjustment_orcahand_v2_calibrated.yaml` next to the original (use `--optimize-fingers index middle ...` to also fit single fingers, `-o` to choose the output). Copy the values you want into `mano_adjustment_orcahand_v2.yaml`.

## Test without the glove

Replay recorded hand poses through the retargeter and show the result in RViz:
```bash
ros2 launch faive_test retargeting_test.launch.py
```
Record your own poses with `python experiments/rokoko_recorder.py` (press Enter to save a pose, `q` to stop), copy the JSON file to `tests/artifacts/`, rebuild, and pass `handposes_file:=<file name>`.
