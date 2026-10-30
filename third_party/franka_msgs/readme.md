## Troubleshooting

### `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'franka_msgs'`

and an undefined symbol like:

`rosidl_typesupport_c__get_message_type_support_handle__franka_msgs__srv__ErrorRecovery_Response`

### Root cause

The conda environment may contain `ros-humble-franka-msgs`, which installs `libfranka_msgs__*.so` with a different interface than this workspace version (`third_party/franka_msgs`).  
At runtime, ROS can load the conda `.so` files first, causing ABI/type-support mismatch.

### Fix (verified)

1. Remove the conflicting conda package:

```bash
conda remove -n ros2 -y ros-humble-franka-msgs
```

