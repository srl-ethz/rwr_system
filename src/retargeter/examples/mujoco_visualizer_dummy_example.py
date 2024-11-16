import numpy as np
import sys
import os
import time
from mujoco_visualizer import GripperVisualizer

gv = GripperVisualizer(sim_model="/home/meberlein/dev/ros2_ws/src/rwr_system/src/viz/models/orca1_hand/hand_orca1.xml")
i = 0
while True:
    # give sinusoidal joint angle commands
    joint_angles =np.rad2deg( np.sin(i/10) * np.pi/2)
    # make it a 11 element array
    joint_angles = np.array([joint_angles] * 16)
    gv.command_joint_angles(joint_angles)
    time.sleep(0.1)
    i += 1
