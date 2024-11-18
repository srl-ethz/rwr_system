import mujoco, mujoco_viewer
import os
import multiprocessing
from threading import Thread
import numpy as np

class MuJoCoVisualizer():
    """
    MuJoCo simulation of the hand
    Designed to be (mostly) drop-in replacable with instances of GripperController
    currently hardcoded for P4 hand
    """

    def __init__(self, sim_model="../../viz/models/faive_hand_p4/hand_p4.xml"):

        self.controller_conn, self.simulation_conn = multiprocessing.Pipe()

        # Create a new process for the simulation
        self.simulation_process = multiprocessing.Process(target=simulation, args=(self.simulation_conn, sim_model))
        self.simulation_process.start()

        self.num_of_joints = 15
        self.joint_pos_array = np.zeros(self.num_of_joints)
        self.contact_map = np.zeros(self.num_of_joints)

    def command_joint_angles(self, joint_angles: np.array):
        """
        Update the joint angles and send the data (joint_angles, contact_map) through the pipe.
        """
        self.joint_pos_array = joint_angles
        # Send both joint angles and contact map together
        self.controller_conn.send((self.joint_pos_array, self.contact_map))



def simulation(simulation_conn, sim_model):
    """
    Simulation setup and loop for the Faive Hand.

    This function has been modified to update hand colors based on contact sensor data.
    """
    # Load model and data
    model = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(__file__), sim_model))
    data = mujoco.MjData(model)

    mode = "window"
    viewer = mujoco_viewer.MujocoViewer(model, data, mode=mode)

    # Scaling factor for the joint commands
    command_scaling = np.ones(15)
    command_scaling[1:] = 0.5

    # Create a thread to listen for commands from the controller
    def command_listener():
        while True:
            joint_angles = simulation_conn.recv()
            hand_joint_num = 15
            assert len(joint_angles) == hand_joint_num, f"Expected {hand_joint_num} joint angles, got {len(joint_angles)}"
            data.ctrl[-len(joint_angles):] = np.deg2rad(joint_angles) * command_scaling

    # Start the command listener thread
    command_listener_thread = Thread(target=command_listener)
    command_listener_thread.daemon = True
    command_listener_thread.start()

    # Simulation loop
    i = 0
    render_every = 10
    while True:
        if i % render_every == 0:
            viewer.render()
        mujoco.mj_step(model, data)
        i += 1
