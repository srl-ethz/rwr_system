from mjcf_urdf_simple_converter import convert
convert("/home/meberlein/dev/ros2_ws/src/rwr_system/src/viz/models/orca2_hand/hand_orca1.xml", "/home/meberlein/dev/ros2_ws/src/rwr_system/src/viz/models/orca2_hand/urdf/orca1.urdf", asset_file_prefix="package://viz/models/orca1_hand/urdf/")
# or, if you are using it in your ROS package and would like for the mesh directories to be resolved correctly, set meshfile_prefix, for example:
# convert("model.xml", "model.urdf", asset_file_prefix="package://viz/models/orca1_hand/urdf/")

