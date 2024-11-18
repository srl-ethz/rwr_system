from re import L
from dynamixel_client import *
import numpy as np
import time
import yaml
import os
import finger_kinematics as fk
from threading import RLock


class MuscleGroup:
    """
    An isolated muscle group comprised of joints and tendons, which do not affect the joints and tendons not included in the group.
    """
    attributes = ["joint_ids", "tendon_ids", "motor_ids", "motor_map", "spool_rad"]
    def __init__(self, name, muscle_group_json: dict):
        self.name = name
        for attr_name in MuscleGroup.attributes:
            setattr(self, attr_name, muscle_group_json[attr_name])
        print(f"Created muscle group {name} with joint ids {self.joint_ids}, tendon ids {self.tendon_ids}, motor ids {self.motor_ids} and spool_rad {self.spool_rad}")

class HandController:
    """
    class specialized for the VGripper
    wraps DynamixelClient to make it easier to access hand-related functions, letting the user think with "tendons" and "joints" instead of "motors"
    
    ## about tendon direction
    Signs for the tendon length is modified before sending to the robot so for the user, it is always [positive] = [actual tendon length increases]
    The direction of each tendon is set by the sign of the `spool_rad` variable in each muscle group
    """
    def __init__(self, port: str = '/dev/ttyUSB0', config_yml: str = "gripper_defs.yaml", calibration: bool = False, maxCurrent: int = 150):
        """
        config_yml: path to the config file, relative to this source file
        """
        baudrate = 3000000

        self.motor_lock = RLock() # lock to read / write motor information

        self._load_musclegroup_yaml(os.path.join(os.path.dirname(os.path.abspath(__file__)), config_yml))

        self.command_lock = RLock() # lock to receive and read angle commands
        self._cmd_joint_angles = np.zeros(self.joint_nr)

        # initialize and connect dynamixels
        self._dxc = DynamixelClient(self.motor_ids, port, baudrate)
        self.connect_to_dynamixels()

        # initialize the joint
        self.init_joints(calibrate=calibration, maxCurrent=maxCurrent)

    def terminate(self):
        '''
        disable torque and disconnect from dynamixels
        '''
        self.disable_torque()
        time.sleep(0.1) # wait for disabling torque
        self.disconnect_from_dynamixels()
        

    def _load_musclegroup_yaml(self, filename):
        """
        load muscle group definitions from a yaml file
        Assumed to only run once, i.e. muscle groups are not changed during runtime
        """
        with open(filename, 'r') as f:
            print(f"reading muscle group definitions from {filename} ...")
            data = yaml.load(f, Loader=yaml.FullLoader)

        self.muscle_groups = []
        for muscle_group_name, muscle_group_data in data['muscle_groups'].items():
            self.muscle_groups.append(MuscleGroup(muscle_group_name, muscle_group_data))
        
        # define some useful variables to make it easier to access tendon information
        attrs_to_get = ["joint_ids", "motor_ids", "tendon_ids", "spool_rad"]
        for attr in attrs_to_get:
            setattr(self, attr, [])
            for muscle_group in self.muscle_groups:
                getattr(self, attr).extend(getattr(muscle_group, attr))
        for attr in attrs_to_get:
            setattr(self, attr, np.array(getattr(self, attr)))

        self.joint_nr = 0
        # run some sanity checks
        for muscle_group in self.muscle_groups:
            self.joint_nr += len(muscle_group.joint_ids)
            assert len(muscle_group.tendon_ids) == len(muscle_group.spool_rad), "spool_rad must be defined for all tendons"
            assert len(muscle_group.motor_map) == len(muscle_group.tendon_ids), "motor_map must be defined for all tendons"
        assert len(self.motor_ids) == len(set(self.motor_ids)), "duplicate tendon ids should not exist"

    def tendon_pos2motor_pos(self, tendon_lengths):
        """ Input: desired tendon lengths
        Output: desired motor positions """
        motor_pos = np.zeros(len(self.motor_ids))
        m_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            m_nr = len(muscle_group.motor_ids)
            t_nr = len(muscle_group.tendon_ids)
            for m_i in range(m_nr):
                m_id = muscle_group.motor_ids[m_i]
                t_i = muscle_group.motor_map.index(m_id)
                motor_pos[m_idx + m_i] = tendon_lengths[t_idx+t_i]/muscle_group.spool_rad[t_i]
            m_idx += m_nr
            t_idx += t_nr
        return motor_pos

    def motor_pos2tendon_pos(self, motor_pos):
        """ Input: motor positions
        Output: tendon lengths """        
        tendon_lengths = np.zeros(len(self.tendon_ids))
        m_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            m_nr = len(muscle_group.motor_ids)
            t_nr = len(muscle_group.tendon_ids)
            for m_i in range(m_nr):
                m_id = muscle_group.motor_ids[m_i]
                t_i = np.where(np.array(muscle_group.motor_map) == m_id)[0]
                for i in t_i:
                    tendon_lengths[t_idx+i] = motor_pos[m_idx+m_i]*muscle_group.spool_rad[i]
            m_idx += m_nr
            t_idx += t_nr
        return tendon_lengths

    def write_desired_motor_pos(self, motor_positions_rad):
        """
        send position command to the motors
        unit is rad, angle of the motor connected to tendon
        """
        with self.motor_lock:
            self._dxc.write_desired_pos(self.motor_ids, motor_positions_rad)


    def write_desired_motor_current(self, motor_currents_mA):
        """
        send current command to the motors
        unit is mA (positive = pull the tendon)
        """
        m_nr = len(motor_currents_mA)
        m_idx = 0
        directions = np.zeros(m_nr)
        for muscle_group in self.muscle_groups:
            for m_id in muscle_group.motor_ids:
                idx = muscle_group.motor_map.index(m_id)
                directions[m_idx] = np.sign(muscle_group.spool_rad[idx])
                m_idx += 1
        with self.motor_lock:
            self._dxc.write_desired_current(self.motor_ids, - motor_currents_mA * directions)
    
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
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[0]

    def get_motor_cur(self):
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[2]

    def get_motor_vel(self):
        with self.motor_lock:
            return self._dxc.read_pos_vel_cur()[1]

    def wait_for_motion(self):
        while not all(self._dxc.read_status_is_done_moving()):
            time.sleep(0.01)

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
        """ Input: joint angles in rad
        Output: motor positions 
        TODO: Extend the calculation of the tendon lengths for every finger. Tip: A clever design can allow for the same formulas for each finger to reduce complexity.
        """
        tendon_lengths = np.zeros(len(self.tendon_ids))
        j_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            t_nr = len(muscle_group.tendon_ids)
            j_nr = len(muscle_group.joint_ids)
            if muscle_group.name == "finger1":
                tendon_lengths[t_idx:t_idx+t_nr] = fk.pose2tendon_finger1(joint_angles[j_idx],joint_angles[j_idx+1])
            elif muscle_group.name == "finger2":
                pass # tendon_lengths[t_idx:t_idx+t_nr] = fk.pose2tendon_finger2(joint_angles[j_idx],joint_angles[j_idx+1])

            # TODO: Extend the calculations here for your own fingers:





            j_idx += j_nr
            t_idx += t_nr
        return self.tendon_pos2motor_pos(tendon_lengths)

    def init_joints(self, calibrate: bool = False, maxCurrent: int = 150):
        """
        Set the offsets based on the current (initial) motor positions
        :param calibrate: if True, perform calibration and set the offsets else move to the initial position
        TODO: Think of a clever way to perform the calibration. How can you make sure that all the motor are in the corect position?
        """

        cal_yaml_fname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cal.yaml")
        cal_exists = os.path.isfile(cal_yaml_fname)

        if not calibrate and cal_exists:

            # Load the calibration file
            with open(cal_yaml_fname, 'r') as cal_file:
                cal_data = yaml.load(cal_file, Loader=yaml.FullLoader)
            self.motor_id2init_pos = np.array(cal_data["motor_init_pos"])

             # Set to current based position control mode
            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            self.write_desired_motor_pos(self.motor_id2init_pos)
            time.sleep(0.01)   
            self.wait_for_motion()

        else: # This will overwrite the current config file with the new offsets and we will lose all comments in the file

            # Disable torque to allow the motors to move freely
            self.disable_torque()
            input("Move fingers to init posiiton and press Enter to continue...")
            
            # TODO: Add your own calibration procedure here, that move the motors to a defined initial position:





            self.motor_id2init_pos = self.get_motor_pos()

            print(f"Motor positions after calibration (0-10): {self.motor_id2init_pos}")
            # Set to current based position control mode
            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            time.sleep(0.2)

            # Save the offsets to a YAML file
            with open(cal_yaml_fname, 'r') as cal_file:
                cal_orig = yaml.load(cal_file, Loader=yaml.FullLoader)

            cal_orig['motor_init_pos'] = self.motor_id2init_pos.tolist()

            with open(cal_yaml_fname, 'w') as cal_file:
                yaml.dump(cal_orig, cal_file, default_flow_style=False)

        self.motor_pos_norm = self.pose2motors(np.zeros(len(self.joint_ids)))

    def write_desired_joint_angles(self, joint_angles: np.array):
        """
        Command joint angles in deg
        :param: joint_angles: [joint 1 angle, joint 2 angle, ...]
        """
        motor_pos_des = self.pose2motors(np.deg2rad(joint_angles)) - self.motor_pos_norm + self.motor_id2init_pos
        self.write_desired_motor_pos(motor_pos_des)
        time.sleep(0.01) # wait for the command to be sent

if __name__ == "__main__" :
    gc = HandController("/dev/ttyUSB0")
    gc.connect_to_dynamixels()

    gc.init_joints(calibrate=True)

    time.sleep(3.0)
