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

## Installation

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
    ```toml
    [target.unix.activation]
    scripts = ["install/setup.bash"]
    ```
    If the build fails after changes, remove `build/`, `install/` and `log/` and build again.
6. For the RL task, download the cube detector's model files once:
    ```bash
    ./src/rwr_system/experiments/rl_inference/pose_estimation/download_models.sh
    ```

### Installation for Task 2
Task 2 uses ROS 2 Humble. You can set it up in your Task 1 workspace, which replaces its Jazzy environment: remove `pixi.toml`, `pixi.lock`, `.pixi/`, `build/`, `install/` and `log/` there, then start at step 2. Or keep Jazzy and use a separate workspace (e.g. `~/rwr_humble_ws`), starting at step 1:

1. Clone the repository into `~/rwr_humble_ws/src` as in step 1 above.
2. Set up the Humble environment. Python is pinned to 3.12: pixi would otherwise install Python 3.14, which some of our dependencies do not support yet.
    ```bash
    cd ~/rwr_humble_ws
    pixi init -c https://prefix.dev/robostack-humble -c https://prefix.dev/conda-forge
    pixi add "python=3.12" ros-humble-desktop ros-dev-tools pip ros-humble-rosidl-typesupport-c ros-humble-joint-state-publisher-gui ros-humble-xacro "pytest<9"
    ```
3. Add to `~/humble_ws/pixi.toml`:
    ```toml
    [activation.env]
    VIRTUAL_ENV = ""   # a virtual env active in your terminal (e.g. set by VS Code) would otherwise break the build

    [target.unix.activation]
    scripts = ["install/setup.bash"]
    ```
4. Follow steps 4 to 6 above in `~/rwr_humble_ws`.

Run nodes with `ros2 run <package> <file>` and launch files with `ros2 launch <package> <launch file>`.

## The hand

Tension the tendons, calibrate the hand and move the joints: see [src/hand_control/README.md](src/hand_control/README.md).

To check the hand without teleop or a policy, `experiments/fixed_command.py` publishes fixed joint poses (edit the poses in the script), and `ros2 run experiments command_gui.py -n 17 -t policy_output` moves each joint with a slider.

## Reinforcement learning: cube spin

Run a policy trained in faive_lab on the hand, detect the cube, and calibrate the camera: see [experiments/README.md](experiments/README.md).

## Imitation learning

For an isolated Linux setup, SSH-friendly launch arguments and raw/processed
data validation, see [the RWR 2026 collection guide](docs/rwr26_teleop.md).

The hand follows the Rokoko glove through the retargeter, and the Franka arm follows the glove's wrist. The Franka arm controller is not part of this repository: it subscribes to `/right/arm/end_effector_pose_cmd` and publishes `/right/arm/end_effector_pose`.

1. **Cameras.** Recording uses [`cube_sorting_task.yaml`](experiments/cfgs/cams/cube_sorting_task.yaml) by default. Check that each OAK-D MXID matches its physical view. For a different camera mapping, pass its full path as `oakd_config:=/path/to/cameras.yaml`. Pass `zed_config:=/path/to/zed.yaml` for a ZED camera. In `controllers_and_cameras.launch.py`, select the files using `OAKD_CAMERA_CFG_NAME` and `ZED_CAMERA_CFG_NAME`. Measure camera extrinsics with `experiments/calibrate_camera_pose.py`.
2. **Retargeter.** Tune the mapping from your hand to the robot hand, see [src/retargeter/README.md](src/retargeter/README.md). The ORCA v2 settings are in `experiments/cfgs/retargeter/`.
3. **Test the teleop** (no recording):
    ```bash
    ros2 launch experiments teleop_rokoko.launch.py
    ```
    A `keyboard_runner` terminal opens: press Enter to start, pause and resume the teleop. Foot pedals that type keys work the same way.
4. **Record demonstrations.** The dataset name defaults to `cube_sorting`. Pass `dataset_name:=NAME` to choose another name.
    ```bash
    ros2 launch experiments record_demonstration.launch.py
    ```
    In the `keyboard_runner` terminal, Enter starts the teleop, starts the recording and stops it. Each demonstration is saved as an `.h5` file in `tmp/<DATASET_NAME>/` (relative to where you launched). The default recorded topics are listed in `experiments/cfgs/logger/datacollection_right.yaml`. Pass `logger_config:=/path/to/logger.yaml` to record a different set of topics.
5. **Process the data.** Resample all topics of the recordings in a folder to a common rate:
    ```bash
    ros2 run logger syncronize_data.py tmp/<DATASET_NAME> --sampling_freq 10 \
      --cfg_file "$(ros2 pkg prefix --share experiments)/cfgs/logger/datacollection_right.yaml"
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
