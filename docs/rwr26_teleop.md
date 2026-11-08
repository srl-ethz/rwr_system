# RWR 2026 teleoperation and recording

This fork keeps its changes on `fix/rwr26-teleop-recording`. Push to your fork;
keep the public course repository as a read-only upstream. Keep camera IDs,
credentials, calibration backups and recordings outside the source checkout.

## Isolated Linux environment

From a shell with Miniforge/mamba available:

```bash
export RWR_WS="$HOME/workspace/rwr26_task2_ws"
mkdir -p "$RWR_WS/src"
git clone https://github.com/Meteors27/rwr_system.git "$RWR_WS/src/rwr_system"
cd "$RWR_WS/src/rwr_system"
git switch fix/rwr26-teleop-recording
git remote add upstream https://github.com/srl-ethz/rwr_system.git
git remote set-url --push upstream DISABLED
mamba env create -p "$RWR_WS/.env" -f environment.teleop.yml
mamba run -p "$RWR_WS/.env" python -m pip install -r requirements-teleop.txt
mamba run -p "$RWR_WS/.env" python -m pip install --no-deps orca_core==0.5.2
mamba run -p "$RWR_WS/.env" python -m pip install --no-deps -e .
cd "$RWR_WS"
mamba run -p "$RWR_WS/.env" colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

As in upstream's README, `orca_core` uses `--no-deps` because its declared NumPy
requirement conflicts with this project's NumPy 1.26.4 pin. The remaining ORCA
dependencies are installed explicitly. Do not downgrade NumPy in an existing
shared ROS environment. The isolated environment was checked with ROS Jazzy,
Python 3.12, CUDA-enabled PyTorch 2.10 and an RTX 3090.

In every runtime terminal, activate this environment and source only this workspace:

```bash
conda activate "$RWR_WS/.env"
source "$RWR_WS/install/setup.bash"
cd "$RWR_WS"
```

## Connect and check each component

1. The Franka Cartesian controller is external to this repository. Obtain its
   actual startup command, robot IP and ROS domain from the station owner.
   It must publish `/right/arm/end_effector_pose` and subscribe to
   `/right/arm/end_effector_pose_cmd`, both `geometry_msgs/msg/PoseStamped`.
   Check its frame/TCP definition and motion limits before enabling teleoperation.
2. Connect the ORCA control USB and power. Use this hand's calibration. A fresh
   controller installation without `calibration.yaml` automatically calibrates
   and moves to neutral at startup; keep clear and have the emergency stop ready.
   Back up calibration before rebuilding/removing `install/`.
3. Rokoko Studio must stream JSON with the right hand, forearm and Coil Pro wrist
   tracking to the Ubuntu host's reachable LAN address, UDP port 14043. Tailscale
   SSH connectivity alone does not establish that the Windows laptop can stream
   to that address.
4. Put camera serials in `$RWR_WS/local/cameras.yaml`, using names `front_view`
   and `wrist_view`. Confirm the names against the actual images, then keep the
   cameras fixed. The right-hand recording config records both RGB streams and
   their intrinsic matrices.

```yaml
# local/cameras.yaml -- replace with actual serial numbers
WORKSPACE_CAMERA_SERIAL:
  name: front_view
WRIST_CAMERA_SERIAL:
  name: wrist_view
```

With two people present, first test teleoperation without the recorder:

```bash
ros2 launch experiments teleop_rokoko.launch.py enable_keyboard_runner:=false
```

In a second, interactive terminal:

```bash
ros2 run experiments keyboard_runner.py --right_hand --retargeter --teleop_pause_and_resume --teleop_back_to_init
```

The first prompt offers to move to a hardcoded initial arm pose. Choose `n` until
that pose and the path to it have been checked at this station. Enter at the next
prompt resumes arm teleoperation and enables the retargeter. Start with small
motions, then verify pause/resume, joint directions and a simple grasp. The hand
retargeter starts disabled in these launch files. Software pause holds the latest
targets; it is not an emergency stop.

Stop the teleop launch before starting the recording launch below: both launch
their own controller, retargeter and Rokoko ingress. If a hand controller is
deliberately kept running separately, use `enable_hand_controller:=false`.

## Record a short demonstration first

### Hand-only operation while the Franka controller is unavailable

To start the glove, retargeter and hand without the arm node or keyboard runner:

```bash
ros2 launch experiments teleop_rokoko.launch.py teleop_arm:=false
```

The retargeter stays disabled initially. After checking the hand's initialization,
enable following in another terminal:

```bash
ros2 service call /right/retargeter/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Pause following with the same command and `{data: false}`. The hand holds its
latest target while paused. To release hand torque before stopping the launch:

```bash
ros2 service call /right/hand/estop std_srvs/srv/SetBool '{data: true}'
```

This hand stop closes the controller connection; restart the launch to use it
again. Hand-only samples lack the arm observations/actions required for the full
task demonstrations.

### Full arm-and-hand demonstrations

```bash
ros2 launch experiments record_demonstration.launch.py \
  oakd_config:="$RWR_WS/local/cameras.yaml" \
  logger_config:="$RWR_WS/src/rwr_system/experiments/cfgs/logger/datacollection_right.yaml" \
  storage_base_path:="$RWR_WS/data/raw" \
  dataset_name:=single_color_session_01 \
  enable_keyboard_runner:=false
```

In the second terminal:

```bash
ros2 run experiments keyboard_runner.py --right_hand --retargeter --teleop_pause_and_resume --teleop_back_to_init --demo_logger
```

Follow the prompts in order: choose whether to move to init, start teleop,
start recording, stop/pause, then save or discard. Wait for HDF5 conversion to
finish before starting another recording. Saving creates a timestamped `.h5`
and removes the temporary rosbag; retain the unsynchronized HDF5 as the raw copy.
Raw uncompressed images use substantial space, so check free storage between
batches and copy verified data to separate storage.

## Validate and synchronize

Inspect a specific raw file (replace the timestamp):

```bash
python src/rwr_system/experiments/scripts/inspect_demonstration.py \
  data/raw/single_color_session_01/TIMESTAMP.h5 \
  --output work/raw_preview
ros2 run logger syncronize_data.py data/raw/single_color_session_01 \
  --sampling_freq 10 --compress \
  --cfg_file "$RWR_WS/src/rwr_system/experiments/cfgs/logger/datacollection_right.yaml"
python src/rwr_system/experiments/scripts/inspect_demonstration.py \
  data/raw/single_color_session_01_processed_10hz_lzf/0000.h5 \
  --output work/processed_preview
```

Both inspections must pass. Review `preview.mp4`, `trajectory.png` and
`summary.json`: confirm views, image motion, complete 7-value arm poses,
17-value hand states/actions, finite values, valid quaternions and plausible
command/observation timing. Synchronization uses ROS receive timestamps and the
shared time interval; it does not establish sensor latency calibration. The
inspector rejects missing streams and long gaps, but a human must still review
tracking quality, frozen images and task success.

Only then collect 5-10 complete task demonstrations and inspect the batch before
increasing its size. Keep a session note with code commit, calibration, camera
placement, operator and successful/failed episode labels.

## Offline verification

No robot or glove driver is started by these tests. Use a separate local ROS
domain; the settings below apply only inside the test command:

```bash
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python -m pytest -q src/rwr_system/tests/offline
```

Tests cover right-only startup, Best Effort RGB recording, intrinsic-message
conversion, HDF5 synchronization/preview, missing-stream rejection, disjoint
time intervals, and finite 17-joint v2 retargeting in hardware joint order.
``pip check`` still reports the documented ORCA/NumPy declaration conflict.
