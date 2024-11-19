import numpy as np
import time
from click import getchar
import yaml
import os
import cvxopt
import faive_system.src.hand_control.simplified_finger_kinematics as fk
import sympy as sym
from threading import Thread, RLock, Event
from .joint_ekf import EKF
from faive_system.src.common.utils import get_datetime_str
from .dynamixel_client_srl import *

np.set_printoptions(precision=3, suppress=True)


class MuscleGroup:
    """
    An isolated muscle group comprised of joints and tendons, which do not affect the joints and tendons not included in the group.
    """

    attributes = [
        "joint_ids",
        "motor_ids",
        "spool_rad",
        "wind_direction",
        "joint_ranges",
        "motor_init_pos",
        "calibration_max",
        "alpha",
        "joint_radius",
    ]

    def __init__(self, name, muscle_group_json: dict):
        self.name = name
        for attr_name in MuscleGroup.attributes:
            setattr(self, attr_name, muscle_group_json[attr_name])

        self.orig_joint_ranges = self.joint_ranges.copy()
        joint_ranges = self.joint_ranges
        for idx in range(len(joint_ranges)):
            low_lim, up_lim = joint_ranges[idx]
            assert (
                low_lim < up_lim
            ), f"joint range [idx] {low_lim} < {up_lim} is invalid"
            mid_pos = (low_lim + up_lim) / 2
            full_range = up_lim - low_lim

        print(
            f"Created muscle group {name} with joint ids {self.joint_ids}, motor ids {self.motor_ids} and spool_rad {self.spool_rad}"
        )

    def build_muscle_groups(config_yml: str):
        with open(config_yml, "r") as f:
            config = yaml.safe_load(f)
        muscle_groups = []
        for name, muscle_group_json in config["muscle_groups"].items():
            muscle_groups.append(MuscleGroup(name, muscle_group_json))
        return muscle_groups


class HandController:
    """
    class specialized for the VHand
    wraps DynamixelClient to make it easier to access hand-related functions, letting the user think with "tendons" instead of "motors"
    eventually, the functionality for joint-level control will also be integrated to this class

    ## about tendon direction
    Signs for the tendon length is modified before sending to the robot so for the user, it is always [positive] = [actual tendon length increases]
    The direction of each tendon is set by the sign of the `spool_rad` variable in each muscle group
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        config_yml: str = "hand_defs.yaml",
        init_motor_pos_update_thread: bool = True,
        compliant_test_mode: bool = False,
        max_motor_current: float = 200.0,
        dummy_mode: bool = False,
        baudrate: int = 3000000
    ):
        """
        config_yml: path to the config file, relative to this source file
        init_motor_pos_update_thread: if True, start a thread to call update_motor_status() pediodically, which updates the motor positions and EKF joint angle estimates.
                                      set to False if you implement your own loop to call update_motor_status(), or you don't need it
        compliant_test_mode: pull on all tendons lightly, disabling position control (useful for checking proprioception etc.)
        max_motor_current: maximum current allowed to be sent to the motors during position control
        """

        self.motor_lock = RLock()
        self.keep_running = Event()
        self.keep_running.set()

        self.operating_mode = -1

        self.init_motor_pos_update_thread = init_motor_pos_update_thread
        self.compliant_test_mode = compliant_test_mode
        self.max_motor_current = max_motor_current
        self.cal_motor_current = 1.0

        self._load_musclegroup_yaml(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), config_yml)
        )

        if dummy_mode:
            self._dxc = DummyDynamixelClient(
                self.motor_ids, port, baudrate, lazy_connect=True
            )
        else:
            self._dxc = DynamixelIndirectClient(
                start_update_thread=True,
                motor_ids=self.motor_ids,
                port=port,
                baudrate=baudrate,
                lazy_connect=True,
            )

        print(f"GC Motor IDs: {self.motor_ids}")
        print(f"DxC Motor IDs: {self._dxc.motor_ids}")

        self.num_of_joints = 0
        pose = []
        for muscle_group in self.muscle_groups:
            self.num_of_joints += len(muscle_group.joint_ids)
            pose.extend(muscle_group.calibration_max)
        self.calib_pose = np.array(pose, dtype=np.float32)
        self.joint_pos_array = np.zeros(self.num_of_joints, dtype=np.float32)
        self.joint_vel_array = np.zeros(self.num_of_joints, dtype=np.float32)

        print(f"Num joints: {self.num_of_joints}")

        self.ekfs = []
        for muscle_group in self.muscle_groups:
            self.ekfs.append(EKF(muscle_group, self))

        self.last_motor_pos_cmd = np.zeros(len(self.motor_ids), dtype=np.float32)
        self.last_joint_angle_cmd = np.zeros(self.num_of_joints, dtype=np.float32)

        self.directions = np.zeros(len(self.motor_ids), dtype=np.int8)
        directions_idx = 0
        for mg in self.muscle_groups:
            for motor_idx in range(len(mg.motor_ids)):
                self.directions[directions_idx] = np.sign(mg.wind_direction[motor_idx])
                directions_idx += 1

    def _load_musclegroup_yaml(self, filename):
        """
        load muscle group definitions from a yaml file
        Assumed to only run once, i.e. muscle groups are not changed during runtime
        """
        # with open(filename, 'r') as f:
        #     print(f"reading muscle group definitions from {filename} ...")
        #     data = yaml.load(f, Loader=yaml.FullLoader)

        # self.muscle_groups = []
        # for muscle_group_name, muscle_group_data in data['muscle_groups'].items():
        #     self.muscle_groups.append(MuscleGroup(muscle_group_name, muscle_group_data))

        self.muscle_groups = MuscleGroup.build_muscle_groups(filename)

        # define some useful variables to make it easier to access tendon information
        attrs_to_get = [
            "joint_ids",
            "motor_ids",
            "spool_rad",
            "wind_direction",
            "motor_init_pos",
        ]
        for attr in attrs_to_get:
            setattr(self, attr, [])
            for muscle_group in self.muscle_groups:
                getattr(self, attr).extend(getattr(muscle_group, attr))
        for attr in attrs_to_get:
            setattr(self, attr, np.array(getattr(self, attr)))

        # run some sanity checks
        assert len(self.motor_ids) == len(
            set(self.motor_ids)
        ), "duplicate motor ids should not exist"

    def tendon_pos2motor_pos_sym(self, tendon_lengths, muscle_group):
        """
        compute for a single muscle group
        Input: desired tendon lengths
        Output: desired motor positions
        """
        motor_pos = sym.matrices.zeros(1, len(muscle_group.motor_ids))
        for m_i, motor_id in enumerate(muscle_group.motor_ids):
            motor_pos[m_i] = tendon_lengths[m_i] / (
                np.abs(muscle_group.spool_rad[m_i]) * muscle_group.wind_direction[m_i]
            )
        return motor_pos

    def write_desired_motor_pos(self, motor_positions_rad):
        """
        send position command to the motors
        unit is rad, angle of the motor connected to tendon
        """
        max_diff = np.max(np.abs(self.last_motor_pos_cmd - motor_positions_rad))
        if max_diff > 0.001:  # approx 0.0572 deg
            self._dxc.gpos = motor_positions_rad
            self.last_motor_pos_cmd = motor_positions_rad.copy()
            print(f"Motor pos cmd: {motor_positions_rad}")
        else:
            # don't write to the motors if the difference is too small
            pass

    def write_desired_motor_current(self, motor_currents_mA):
        """
        send current command to the motors
        unit is mA (positive = pull the tendon)
        """
        with self.motor_lock:
            self._dxc.write_desired_current(
                self.motor_ids, -motor_currents_mA * self.directions
            )

    def connect_to_dynamixels(self):
        with self.motor_lock:
            self._dxc.connect()

    def disconnect_from_dynamixels(self):
        with self.motor_lock:
            self._dxc.disconnect()

    def set_operating_mode(self, mode):
        """
        see dynamixel_client.py for the meaning of the mode
        """
        with self.motor_lock:
            self._dxc.set_operating_mode(self.motor_ids, mode)

    def get_motor_pos(self):
        return self._dxc.ppos

    def get_motor_cur(self):
        return self._dxc.pcurr

    def get_motor_vel(self):
        return self._dxc.pvel

    def _get_motor_pos_vel_cur(self):
        return self._dxc.ppos, self._dxc.pvel, self._dxc.pcurr

    def wait_for_motion(self):
        reached_pos = False
        while not reached_pos:
            if all(self._dxc.read_status_is_done_moving()):
                reached_pos = True

    def enable_torque(self, motor_ids=None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self.motor_lock:
            self._dxc.set_torque_enabled(motor_ids, True)

    def disable_torque(self, motor_ids=None):
        if motor_ids is None:
            motor_ids = self.motor_ids
        with self.motor_lock:
            self._dxc.set_torque_enabled(motor_ids, False)

    def pose2motors(self, joint_angles):
        """Input: joint angles
        Output: motor positions"""
        motor_positions = np.zeros(len(self.motor_ids))
        motor_pos_begin_idx = 0
        joint_pos_begin_idx = 0
        for ekf in self.ekfs:
            # the EKF has lambdified functions (which can be computed really fast) of pose2motors for each muscle group
            # so use that for fast computation
            motor_pos_end_idx = motor_pos_begin_idx + ekf.n_motors
            joint_pos_end_idx = joint_pos_begin_idx + ekf.n_joints
            motor_positions[motor_pos_begin_idx:motor_pos_end_idx] = ekf.pose2motors(
                *joint_angles[joint_pos_begin_idx:joint_pos_end_idx]
            )
            motor_pos_begin_idx = motor_pos_end_idx
            joint_pos_begin_idx = joint_pos_end_idx
        return motor_positions

    def pose2motors_sym(self, joint1, joint2, joint3, joint4=None, muscle_group=None):
        """
        return symbolic function for joint position -> motor position, for a single muscle group (i.e. single finger)
        Input: joint angles
        Output: motor positions
        """
        tendon_lengths = fk.get_tendon_lengths_lambda(
            joint1, joint2, joint3, muscle_group
        )
        motor_pos = self.tendon_pos2motor_pos_sym(
            tendon_lengths, muscle_group=muscle_group
        )
        return motor_pos

    def update_motorinitpos(self):
        """
        Updates the initial motor positions based on the current position
        """
        cal_yaml_fname = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "cal.yaml"
        )

        # get current motor positions
        self.motor_init_pos = self.get_motor_pos()
        print(
            f"Setting current motor position as motor_init_pos: {self.motor_init_pos}"
        )

        # Save the offsets to a YAML file
        cal_data = {}
        cal_data["motor_init_pos"] = self.motor_init_pos.tolist()
        with open(cal_yaml_fname, "w") as cal_file:
            yaml.dump(cal_data, cal_file, default_flow_style=False)

    def init_joints(self, calibrate: bool = False):
        """
        Set the offsets based on the current (initial) motor positions
        :param calibrate: if True, perform calibration and set the offsets else move to the initial position
        """

        calib_current = 60  # mA

        cal_yaml_fname = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "cal.yaml"
        )
        cal_exists = os.path.isfile(cal_yaml_fname)

        if not calibrate and cal_exists:

            # Load the calibration file
            with open(cal_yaml_fname, "r") as cal_file:
                cal_data = yaml.load(cal_file, Loader=yaml.FullLoader)
            self.motor_init_pos = np.array(cal_data["motor_init_pos"])

            # TODO(@gavincangan): Figure out if we want to save this directly to the config file
            # This will overwrite the current config file with the new offsets and we will lose all comments in the file
        else:
            # Disable torque to allow the motors to move freely
            self.disable_torque()
            input("Move fingers to init position and press Enter to continue...")
            time.sleep(1)  # give some time to hold fingers
            self.enable_torque()

            # Set to current control mode and pull on all tendons (while the user holds the fingers in the init position)
            self.set_operating_mode(0)
            self.write_desired_motor_current(
                calib_current * np.ones(len(self.motor_ids))
            )
            time.sleep(2)

            # after pulling for a while, set the motor_init_pos
            self.update_motorinitpos()

        self.motor_pos_norm = self.pose2motors(np.deg2rad(self.calib_pose))
        if self.init_motor_pos_update_thread:
            # start a thread to update the motor positions
            self.motor_pos_update_thread = Thread(target=self._update_motor_status_loop)
            self.motor_pos_update_thread.start()

        if self.compliant_test_mode:
            print("Setting compliant test mode! This disables position control.")
            self.set_operating_mode(0)
            self.write_desired_motor_current(20 * np.ones(len(self.motor_ids)))
        else:
            # start position control
            self.set_operating_mode(5)
            self.write_desired_motor_current(
                self.max_motor_current * np.ones(len(self.motor_ids))
            )
            print(f"Move to init pose: {self.motor_init_pos}")
            self.write_desired_motor_pos(self.motor_init_pos)

    def command_joint_angles(self, joint_angles_deg: np.array):
        """
        Command joint angles
        :param: joint_angles_deg: [joint 1 angle, joint 2 angle, ...]
        """
        max_diff = np.max(np.abs(self.last_joint_angle_cmd - joint_angles_deg))
        if max_diff > 0.1:  # 0.1 deg
            print(f"Joint pos cmd: {joint_angles_deg}")
            motor_pos_des = (
                self.pose2motors(np.deg2rad(joint_angles_deg))
                - self.motor_pos_norm
                + self.motor_init_pos
            )
            self.write_desired_motor_pos(motor_pos_des)
            self.last_joint_angle_cmd = joint_angles_deg.copy()
        else:
            # ignore cmd if the difference is too small
            pass

    def _update_motor_status_loop(self):
        while self.keep_running.is_set():
            time.sleep(0.005)
            self.update_motor_status()

    def update_motor_status(self):
        """
        Update the motor angles and joint angle estimates.
        """
        motor_pos_array, motor_vel_array, _ = self._get_motor_pos_vel_cur()
        motor_pos_array += self.motor_pos_norm - self.motor_init_pos
        motor_pos_begin_idx = 0
        joint_pos_begin_idx = 0
        for ekf in self.ekfs:
            # this assumes that the joints and motors go in order 0, 1, ... in the hand_def
            motor_pos_end_idx = motor_pos_begin_idx + ekf.n_motors
            joint_pos_end_idx = joint_pos_begin_idx + ekf.n_joints
            # update pos and vel
            (
                self.joint_pos_array[joint_pos_begin_idx:joint_pos_end_idx],
                self.joint_vel_array[joint_pos_begin_idx:joint_pos_end_idx],
            ) = ekf.update(
                motor_pos_array[motor_pos_begin_idx:motor_pos_end_idx],
                motor_vel_array[motor_pos_begin_idx:motor_pos_end_idx],
            )
            motor_pos_begin_idx = motor_pos_end_idx
            joint_pos_begin_idx = joint_pos_end_idx

    def get_joint_pos(self):
        return self.joint_pos_array.copy()

    def get_joint_vel(self):
        return self.joint_vel_array.copy()

    def calculate_Jacobian(self, p, q):
        """Input: array of expressions of motor positions, array of controllable joint angles
        Output: jacobian"""
        dim_p = len(p)
        dim_q = len(q)
        J = sym.matrices.zeros(dim_p, dim_q)
        for i in range(dim_p):
            for j in range(dim_q):
                J[i, j] = sym.diff(p[i], q[j])
        return sym.sympify(J)

    def pose2jacobian(self, theta_MCP, theta_PIP, J_func):
        """Input: controllable joint angles
        Output: functionalized jacobian for given joint angles"""
        return J_func(theta_MCP, theta_PIP)

    def compute_mot_torque(self, pose, tau, J_func):
        """Input:
        pose: 2D vector [theta_MCP, theta_PIP]
        tau: 2D vector of generalized forces (torques)
        solves QP to compute muscle force from generalized forces
        QP is formulated to
        - minimize squared sum of muscle tension
        - each muscle tension is negative
        - achieve generalized force
        Output: motor_torques"""
        num_motors = 3
        min_torque = 20  # mA. Minimum torque of motors. Use this to avoid slack wires.
        max_torque = 60
        # define matrices that are constant in the QP formulation
        P = cvxopt.matrix(np.identity(num_motors)) * 1.0
        p = cvxopt.matrix(np.zeros(num_motors)) * 1.0
        G = cvxopt.matrix(np.identity(num_motors)) * -1.0
        # TODO: Adjust G matrix for motor directions!
        h = -min_torque * cvxopt.matrix(np.ones(num_motors)) * 1.0
        cvxopt.solvers.options["show_progress"] = False  # suppress output
        assert len(pose) == 2
        assert len(tau) == 2
        Jacobian = self.pose2jacobian(pose[0], pose[1], J_func)
        # define matrices for the equality constraint
        A = cvxopt.matrix(np.transpose(Jacobian)) * 1.0
        b = cvxopt.matrix(tau) * 1.0
        # solve QP
        sol = cvxopt.solvers.qp(P, p, G, h, A, b)
        for i in range(len(sol["x"])):
            if sol["x"][i] > max_torque:
                sol["x"][i] = max_torque
        return sol["x"]

    def stop(self):
        self.keep_running.clear()
        if self.init_motor_pos_update_thread:
            self.motor_pos_update_thread.join()


if __name__ == "__main__":
    gc = HandController("/dev/ttyUSB0")
    # gc.connect_to_dynamixels()

    gc.init_joints(calibrate=False)

    time.sleep(3.0)
