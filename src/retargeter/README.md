# Faive mano retarget ros

The ros package for retargeting mano hand model to the hand

## Discretized retargeter

`discretized_retargeter_node.py` reads the JSON written by
`retargeter_collect_calibration_data.py`. It removes global MANO translation and rotation,
matches the 20 finger keypoints to the closest calibrated sample, and publishes that sample's
robot joint positions only after a debounced posture transition. The first observed posture and
every `set_enabled(true)` request establish a silent baseline.

`experiments/launch/hil_rl.launch.py` selects this retargeter automatically. Pass
`left_retargeter_calibration_file` and `right_retargeter_calibration_file`; an empty path selects
the newest chirality-matching calibration file next to the retargeter source/executable.

## Get started

In your ros2 workspace clone the repo of faive_system

```bash
# the following command builds both faive_mano_retarget and faive_mano_retarget_ros
colcon build  --symlink-install --packages-up-to faive_mano_retarget_ros
# the following command builds the package for the model, visulization utils, etc
colcon build  --symlink-install --packages-up-to viz
```

With the rokoko json streaming on, open a terminal and run the retargeter

```bash
source install/setup.bash
ros2 launch faive_mano_retarget_ros rokoko_retarget_launch.yaml
```

The rokoko_node communicate with the following topics


- Subscribers: 
- Publishers:
    - `/faive/policy_output`: `std_msgs/msg/Float32MultiArray` The degrees of each tendons, dim=15 for p4 hand. Note that for rolling contact joints, the values are the rad of two virtual joints added together.
    - `/ingress/mano`: `std_msgs/msg/Float32MultiArray` The un-normalized raw mano points.
    - `/ingress/wrist`: `geometry_msgs/msg/PoseStamped` The writst pose
    - `/parameter_events`: `rcl_interfaces/msg/ParameterEvent`
    - `/retarget/normalized_mano_points`: `visualization_msgs/msg/MarkerArray` (debug only), mano points in a hand shape.
    - `/rosout`: `rcl_interfaces/msg/Log`

Then launch the viualize_joints and rviz for visulization

```bash
source install/setup.bash
ros2 launch viz p4_viz_launch.py
```

Then manual add marker array and robot model in rviz, and change the map frame to `root`
