# RWR System

Code for the [Real-World Robotics](https://rwr.ethz.ch/) class (RWR 2026) with the ORCA v2 hand:

- **Reinforcement learning:** run a policy trained in simulation on the real hand (cube spin), with camera-based cube detection.
- **Imitation learning:** teleoperate the hand (Rokoko glove) and the Franka arm, record demonstrations, and run the trained policy.

The code of the RWR 2024 class is kept under the git tag [`rwr2024`](https://github.com/srl-ethz/rwr_system/tree/rwr2024).

## Repository layout

| Folder | ROS package | Content |
|---|---|---|
| `src/hand_control` | `hand_control` | ORCA v2 hand controller (through [orca_core](https://pypi.org/project/orca_core/)), Franka gripper, tensioning |
| `src/ingress` | `ingress` | Sensor input: OAK-D and ZED cameras, Rokoko glove, webcam hand tracking (MediaPipe) |
| `src/retargeter` | `retargeter` | Maps the human hand keypoints to the hand's joint angles, with tools to tune and calibrate it |
| `src/logger` | `logger` | Records ROS topics during demonstrations and converts them to HDF5 |
| `src/viz` | `viz` | Robot models (ORCA v2, Franka, cube), RViz configs, joint visualization |
| `src/common` | | Shared Python utilities |
| `experiments` | `experiments` | Launch files, configs and scripts for the tasks |
| `tests` | `faive_test` | Retargeter replay test, ORCA v2 chirp tests |
| `third_party/franka_msgs` | `franka_msgs` | Franka message definitions |

The Python package is called `faive_system` (e.g. `from faive_system.src.common.utils import ...`).

## Installation (Linux)

1. Create the `src` folder of your ROS 2 workspace and clone the repository into it:
   ```bash
     mkdir -p ~/ros_ws/src
     cd ~/ros_ws/src
     git clone https://github.com/srl-ethz/rwr_system.git
   ```
2. Set up a ROS 2 Jazzy environment, e.g. with pixi following [Getting Started with RoboStack](https://robostack.github.io/GettingStarted.html). With pixi, prefix the commands below with `pixi run` and run them from `~/ros_ws`.
3. Install the ROS dependencies:
   ```bash
     pixi add pip ros-jazzy-rosidl-typesupport-c ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
     pixi run which pip   # should print ~/ros_ws/.pixi/envs/default/bin/pip or similar
   ```

     If `which pip` prints another pip, comment out the conda/mamba setup in `~/.bashrc` or `~/.zshrc`.
4. Install the Python dependencies:
   ```bash
     pixi run pip install -e './src/rwr_system[all]'
     # the ORCA v2 driver. --no-deps: orca_core asks for a newer numpy than this package pins, but works with it
     pixi run pip install --no-deps orca_core==0.5.2
   ```
5. Build the workspace and source it:
   ```bash
     cd ~/ros_ws
     pixi run colcon build --symlink-install
     source install/setup.bash
   ```

     With pixi, add the source step to `~/ros_ws/pixi.toml` instead:

     If the build fails after changes, remove `build/`, `install/` and `log/` and build again.
6. For the RL task, download the cube detector's model files once:
   ```bash
     ./src/rwr_system/experiments/rl_inference/pose_estimation/download_models.sh
   ```

Run nodes with `ros2 run <package> <file>` and launch files with `ros2 launch <package> <launch file>`.

### Installation (Apple Silicon/macOS)

The commands to run the system are the same as on Linux; the code selects the platform's libraries and GPU by itself. Only the installation differs:

1. Clone the repository as in step 1 above, then create the environment (ROS 2 Jazzy, PyTorch and the other dependencies) with [micromamba](https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html):
   ```bash
     cd ~/ros_ws/src/rwr_system
     micromamba create -f environment-macos.yml
     micromamba activate rwr_jazzy
     pip install --no-deps -e .
   ```
2. Build the workspace and source it. In zsh, use `setup.zsh`:
   ```bash
     cd ~/ros_ws
     colcon build --symlink-install --packages-select ingress hand_control viz experiments
     source install/setup.zsh
   ```

     If the repository is not in `~/ros_ws/src`, add `--base-paths /path/to/rwr_system` to the build command. In each new terminal, run `micromamba activate rwr_jazzy` and the `source` line again.
3. For the RL task, download the cube detector's model files (step 6 above).

Differences to Linux:

- **DDS:** the environment sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, because FastDDS delivers no camera images with macOS's default UDP limits. All machines in a ROS network must use the same setting.
- **Cube detector:** it runs on the Mac's GPU (MPS). A benchmark of the model on an M4 reached about 5.5 Hz, far below the 20 Hz of the policy, so use the Linux robot PC for the full cube spin system. To check the rate with a cube in view: `ros2 topic hz /cube_detection_node/cube_markers`.
- **Serial port:** the Linux low-latency setting of the serial port may not apply on macOS. Check with `ros2 topic hz /right/hand/joint_position_from_motor` that the joint state rate is high enough. The hand's controller board appears as two `usbmodem` ports; `port: auto` selects the motor port.
- **Message libraries:** macOS removes `DYLD_LIBRARY_PATH` from nodes started through `#!/usr/bin/env python3`, which makes this workspace's message types (e.g. `ingress.msg`) fail to load with `invalid allocator` or `type_support is null`. A node that uses them must call `restore_library_path_on_macos()` from `faive_system.src.common.ros_utils` at the top of the script. It does nothing on Linux.

For the chirp recorder on macOS, see [experiments/orca/README.md](experiments/orca/README.md).

## The hand

Do these steps in order after the assembly, and repeat them from step 1 when the tendons are slack. Only one program can use the hand's serial port at a time, so stop each step before you start the next. Source the workspace first (see Installation). Run the `python` commands from the repository root.

Make sure that the servo wiring is correct before you connect the power: the servos have no reverse-polarity protection.

1. **Tension the tendons.** The motors wind the tendons taut and then hold them under current. With the 3D-printed ratchet, turn the top spool of each servo clockwise until the tendon feels firm, and no further: too much tension makes joints stall before their end stops in calibration. Press Ctrl+C when done:
   ```bash
     python src/hand_control/orca/tensioning_v2.py
   ```

     `Motor ID N has not been fully calibrated` for all motors is normal here: this script reads the source config folder, and the calibration is in `install/`. Tensioning does not use it.
2. **Calibrate the hand** once after tensioning. The joints move to their end stops, so keep your fingers clear:
   ```bash
     ros2 launch hand_control orcahand_v2_controller_launch.yaml hand_chirality:=right calibrate:=true launch_rviz:=false
   ```

     The result is saved as `calibration.yaml` next to the installed config, so deleting `install/` means calibrating again. When it is finished, press Ctrl+C and compare the measured motor travel of equivalent joints:
   ```bash
     sed -n '/motor_travel_measured/,$p' ~/ros_ws/install/hand_control/share/hand_control/orca/configs/orcahand_v2/right/calibration.yaml
   ```

     `calibrated: true` does not show that the result is good. A joint far below its neighbours stopped before its end stop (on our hand, the PIP joints measure about 90 to 100 degrees), see Troubleshooting in [experiments/README.md](experiments/README.md). Fix it and calibrate again before you continue.
3. **Move each joint** with the sliders and check that it moves correctly and over its full range:
   ```bash
     ros2 launch experiments orcahand_v2_joint_gui.launch.py
   ```

     Then stop the GUI.
4. **Record chirps for system identification.** The recorder uses `orca_core` directly, without ROS. Remove all loads from the hand and leave it room to move. See [experiments/orca/README.md](experiments/orca/README.md) for the details.
   ```bash
     # copy the config and the calibration to their own folder; repeat after each calibration or config change
     mkdir -p ~/ros_ws/chirp_config
     cp -L ~/ros_ws/install/hand_control/share/hand_control/orca/configs/orcahand_v2/right/{config,calibration}.yaml ~/ros_ws/chirp_config/

     # show the plan without moving, then test one joint with a small amplitude
     python experiments/orca/chirp.py --config ~/ros_ws/chirp_config/config.yaml --joints index_mcp --dry-run
     python experiments/orca/chirp.py --config ~/ros_ws/chirp_config/config.yaml --joints index_mcp --amplitude-deg 3 --execute

     # record the 8 joint types
     python experiments/orca/chirp.py --config ~/ros_ws/chirp_config/config.yaml \
       --joints wrist thumb_cmc thumb_abd thumb_mcp thumb_dip index_abd index_mcp index_pip \
       --ctrl-freq 20 --duration 20 --f0 0.1 --f1 5 --amplitude-deg 10 --execute
   ```

     Each run creates a folder in `logs/chirp_v2/`. Use it only if `metadata.json` says `complete` and each CSV has 400 rows. Plot a recording with `python experiments/orca/plot.py <recording.csv> --save`.
5. **Fit the joint parameters** on the training computer, in the Python environment with Isaac Lab, from `scripts/motor_param_characterization` in `rwr_faive_lab`. Install the fit's packages once with `python -m pip install scikit-optimize pandas`. The simulator's joint names have the prefix `right_`:
   ```bash
     for j in wrist thumb_cmc thumb_abd thumb_mcp thumb_dip index_abd index_mcp index_pip; do
       python optimization.py --joint_name right_$j --ground_truth <run_dir>/chirp_${j}_*.csv
     done
   ```

     The output shows the best parameters, and `logs/optimizations/<joint>_<timestamp>/frf_comparison.png` compares them with the real hand. If a best value is at the limit of its range, make the range larger in `optimization.py` and fit again. If you change the chirp on the real hand, also change it in `chirp_characterization.py`.
6. **Use the parameters.** Write them into `source/faive_lab/faive_lab/assets/orca_v2.py` in `rwr_faive_lab` (the index values also apply to the middle, ring and pinky fingers), then train the policy again.

If something fails, see Troubleshooting in [experiments/README.md](experiments/README.md).

To check the hand without teleop or a policy, `experiments/fixed_command.py` publishes fixed joint poses (edit the poses in the script), and `ros2 run experiments command_gui.py -n 17 -t policy_output` moves each joint with a slider.

## Reinforcement learning: cube spin

Run a policy trained in faive_lab on the hand, detect the cube, and calibrate the camera: see [experiments/README.md](experiments/README.md).

## Imitation learning

The hand follows the Rokoko glove through the retargeter, and the Franka arm follows the glove's wrist. The Franka arm controller is not part of this repository: it subscribes to `/right/arm/end_effector_pose_cmd` and publishes `/right/arm/end_effector_pose`.

1. **Cameras.** Set the ID printed on each OAK-D camera in a file in `experiments/cfgs/cams/` and select it with `OAKD_CAMERA_CFG_NAME` in `record_demonstration.launch.py` and `controllers_and_cameras.launch.py` (default: `oakd_scissors_task.yaml`). For a ZED camera, set `ZED_CAMERA_CFG_NAME`. The camera extrinsics can be measured with `experiments/calibrate_camera_pose.py`.
2. **Retargeter.** Tune the mapping from your hand to the robot hand, see [src/retargeter/README.md](src/retargeter/README.md). The ORCA v2 settings are in `experiments/cfgs/retargeter/`.
3. **Test the teleop** (no recording):
   ```bash
     ros2 launch experiments teleop_rokoko.launch.py
   ```

     A `keyboard_runner` terminal opens: press Enter to start, pause and resume the teleop. Foot pedals that type keys work the same way.
4. **Record demonstrations.** Set `DATASET_NAME` in `record_demonstration.launch.py`, then:
   ```bash
     ros2 launch experiments record_demonstration.launch.py
   ```

     In the `keyboard_runner` terminal, Enter starts the teleop, starts the recording and stops it. Each demonstration is saved as an `.h5` file in `tmp/<DATASET_NAME>/` (relative to where you launched). The recorded topics are listed in `experiments/cfgs/logger/datacollection_default.yaml`.
5. **Process the data.** Resample all topics of the recordings in a folder to a common rate:
   ```bash
     ros2 run logger syncronize_data.py tmp/<DATASET_NAME> --sampling_freq 10
   ```

     Check the demonstrations with `experiments/scripts/plot_trajectories.py` and `experiments/scripts/visualize_videos.py`.
6. **Train** a policy, e.g. with [srl_il](https://github.com/srl-ethz/srl_il).
7. **Run the policy.** Start the cameras and the hand, then the policy:
   ```bash
     ros2 launch experiments controllers_and_cameras.launch.py
     ros2 launch experiments run_policy_inference.launch.py policy_ckpt_path:=/path/to/policy.ckpt
   ```

     The inference config (`experiments/cfgs/inference_config_*.yaml`, set with `inference_cfg_file:=`) maps the camera and joint topics to the policy's inputs and outputs; it must match your training data.

## Other tools

| Tool | Use |
|---|---|
| `experiments/gesture_runner.py` | Enables the retargeter only while one wrist is held more than 10 cm higher than the other (needs both gloves), so the hand stops following when you lower it |
| `experiments/run_keyboard_teleop_franka.py` | Moves the Franka end effector with the keyboard |
| `experiments/calibrate_franka_load.py` | Calibrates the load mounted on the Franka |
| `src/viz/visualize_static_frames.py` | Publishes fixed frames (e.g. cameras) from a YAML file such as `experiments/cfgs/tf/static_frames_bimanual.yaml` |
| `experiments/orca/` | ORCA v2 chirp data acquisition for system identification, see [experiments/orca/README.md](experiments/orca/README.md) |

## Contributing

To contribute, fork the repository and open a pull request.
