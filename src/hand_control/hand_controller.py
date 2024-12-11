from re import L
import numpy as np
import time
import yaml
import os
from threading import RLock
import faive_system.src.hand_control.finger_kinematics as fk
from faive_system.src.hand_control.dynamixel_client import *
from calibration import CalibrationClass

class MuscleGroup:
    """
    An isolated muscle group comprised of joints and tendons, which do not affect the joints and tendons not included in the group.
    """
    attributes = ["joint_ids", "joint_roms", "tendon_ids", "motor_ids", "motor_map", "spool_rad"]
    def __init__(self, name, muscle_group_json: dict):
        self.name = name
        for attr_name in MuscleGroup.attributes:
            setattr(self, attr_name, muscle_group_json[attr_name])
        print(f"Created muscle group {name} with joint ids {self.joint_ids}, tendon ids {self.tendon_ids}, motor ids {self.motor_ids} and spool_rad {self.spool_rad}")

class HandController(CalibrationClass):
    """
    class specialized for Hand Control of the Orca Hand.
    wraps DynamixelClient to make it easier to access hand-related functions, letting the user think with "tendons" and "joints" instead of "motors"
    
    ## about tendon direction
    Signs for the tendon length is modified before sending to the robot so for the user, it is always [positive] = [actual tendon length increases]
    The direction of each tendon is set by the sign of the `spool_rad` variable in each muscle group
    """

    def __init__(self, port: str = '/dev/ttyUSB0', config_yml: str = "hand_defs.yaml", calibration: bool = False, auto_calibrate: bool = False, maxCurrent: int = 150):
        """
        config_yml: path to the config file, relative to this source file
        """
        
        ### All configurations are here ### 

        maxCurrent = 200
        calibration_current = 200
        
        baudrate = 3000000

        # Mapping of joint names to their index ranges from the joint_angles array
        self.mano_joint_mapping = {
            "wrist": slice(0, 1),   # 0 --> wrist pitch
            "thumb": slice(1, 5),   # 1,2,3,4 --> thumb [ABD,MCP,PIP,DIP] 
            "index": slice(5, 8),   # 5,6,7 --> [ABD,MCP,PIP]
            "middle": slice(8, 11), # 8,9,10 
            "ring": slice(11, 14),  # 11,12,13 
            "pinky": slice(14, 17), # 14,15,16    
        }

        self.motor_lock = RLock() # lock to read / write motor information

        self._load_musclegroup_yaml(os.path.join(os.path.dirname(os.path.abspath(__file__)), config_yml))
        
        self.command_lock = RLock() # lock to receive and read angle commands
        self._cmd_joint_angles = np.zeros(self.joint_nr)

        # initialize and connect dynamixels
        # self._dxc = DummyDynamixelClient(self.motor_ids, port, baudrate)
        self._dxc = DynamixelClient(self.motor_ids, port, baudrate)

        self.connect_to_dynamixels()

        self.motor_ids_dict = self.get_motor_id_dict() 
        
        self.mano_joints_rom_list = self.get_mano_joints_rom_list()
        
        # self.mano_motor_directions = self.get_mano_motor_directions()
        
        # If we have auto_calibrate no manual calibration is needed

        # Map Mano indexes to the corresponding motor_ids index
        self.mano_to_motor_ids_mapping = self.get_mano_to_motor_ids_mapping()
        
        if auto_calibrate:
            calibration = False
            self.init_joints(calibrate=calibration, auto_calibrate=auto_calibrate, calib_current=calibration_current, maxCurrent=maxCurrent)
        else:
            self.mano_joints2spools_ratio = self.get_joints2spool_ratio()
            self.init_joints(calibrate=calibration, auto_calibrate=auto_calibrate, calib_current=calibration_current, maxCurrent=maxCurrent)
        



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
        attrs_to_get = ["joint_ids", "joint_roms", "motor_ids", "tendon_ids", "spool_rad"]
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
        # Tendon lengths are more than the motor positions
        # we skip every other tendon length (depending on motor_map) as moving the one will automatically set the other
        # so doesn't make sense to calculate the motor position from the tendon length again.
        motor_pos = np.zeros(len(self.motor_ids))
        m_idx = 0
        t_idx = 0
        for muscle_group in self.muscle_groups:
            m_nr = len(muscle_group.motor_ids)
            t_nr = len(muscle_group.tendon_ids)
            for m_i in range(m_nr):
                m_id = muscle_group.motor_ids[m_i]
                t_i = muscle_group.motor_map.index(m_id) # .index() returns the value of the first occurence in the list
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
                t_i = np.where(np.array(muscle_group.motor_map) == m_id)[0] # Get both index responding to that motor_id
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

    # def get_mano_motor_directions(self):
    #     """
    #     Get the ROM (Range of Motion) for each muscle group.
    #     :return: Dictionary with muscle group names as keys and tuples (lower ROM, upper ROM) as values.
    #     """
    #     directions_list = [0 for _ in range(17)]
    #     for muscle_group in self.muscle_groups:
    #         joint_indices = self.mano_joint_mapping[muscle_group.name]

    #         spool_rad_list = muscle_group.spool_rad

    #         # idx is for geting the values from the muscle_roms which start from 0
    #         # joint_idx is for setting the values in the joints_rom_list which depends on the joint 
    #         for idx, joint_idx in enumerate(range(joint_indices.start,joint_indices.stop)):
    #             directions_list[joint_idx] = np.sign(spool_rad_list[idx*2])

    #     return directions_list

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

    def set_operating_mode_for_motors(self, motor_ids, mode):
        """
        Set the operating mode for specific motors.
        :param motor_ids: List of motor IDs to set the mode for.
        :param mode: The operating mode to set.
        """
        with self.motor_lock:
            self._dxc.set_operating_mode(motor_ids, mode)

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
        """
        # return 
        tendon_lengths = np.zeros(len(self.tendon_ids))
        tendon_id_ranges_dict = self.get_tendon_id_ranges()
        for muscle_group in self.muscle_groups:
            t_s, t_e = tendon_id_ranges_dict[muscle_group.name]
                
            joint_indices = self.mano_joint_mapping[muscle_group.name]
            # Extract and return the angles for the specified joint
            joint_angles_of_muscle_group = joint_angles[joint_indices]

            if muscle_group.name == "wrist":
                tendon_lengths[t_s:t_e+1] = fk.pose2wrist(*joint_angles_of_muscle_group)
            else:
                tendon_lengths[t_s:t_e+1] = fk.pose2tendon_length(muscle_group.name, *joint_angles_of_muscle_group)
        return self.tendon_pos2motor_pos(tendon_lengths)

    def get_tendon_id_ranges(self):
        """
        Create a dictionary with the starting tendon id and last tendon id of each muscle group.
        :return: Dictionary with muscle group names as keys and tuples (start_tendon_idx, end_tendon_idx) as values.
        """
        tendon_id_ranges = {}
        for muscle_group in self.muscle_groups:
            start_tendon_idx = muscle_group.tendon_ids[0]-1
            end_tendon_idx = muscle_group.tendon_ids[-1]-1
            tendon_id_ranges[muscle_group.name] = (start_tendon_idx, end_tendon_idx)
        return tendon_id_ranges
    
    
    def get_motor_id_dict(self):
        """
        Create a dictionary with the motor ids for each muscle group.
        :return: Dictionary with muscle group names as keys and lists of motor ids as values.
        """
        motor_id_dict = {}
        for muscle_group in self.muscle_groups:
            motor_id_dict[muscle_group.name] = muscle_group.motor_ids
        return motor_id_dict

    def init_joints(self, calibrate: bool = False, auto_calibrate: bool = False, calib_current: int = 70, maxCurrent: int = 150):
        """
        Set the offsets based on the current (initial) motor positions
        :param calibrate: if True, perform calibration and set the offsets else move to the initial position
        """
        self.motor_pos_norm = np.zeros(len(self.joint_ids))
        
        # Find calibration file
        cal_yaml_fname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cal.yaml")
        cal_exists = os.path.isfile(cal_yaml_fname)

        if not auto_calibrate and not calibrate and cal_exists:
            # Load the calibration file
            with open(cal_yaml_fname, 'r') as cal_file:
                cal_data = yaml.load(cal_file, Loader=yaml.FullLoader)
            self.motor_id2init_pos = np.array(cal_data["motor_init_pos"])

             # Set to current based position control mode
            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            # This is in order to grecefully reached the desired position when starting the hand
            joint_angles = np.zeros(len(self.motor_ids)) 
            self.write_desired_joint_angles(joint_angles, calibrate=True)
            # self.write_desired_motor_pos(self.motor_id2init_pos)
            time.sleep(0.1)   

        else: # This will overwrite the current config file with the new offsets and we will lose all comments in the file
            if auto_calibrate:
                self.auto_calibrate_fingers_with_pos(calib_current, maxCurrent) # Demo script for calibrating based on hardstops of design
                time.sleep(0.3)

                # Load the calibration file that have just been written by the previous function
                with open(cal_yaml_fname, 'r') as cal_file:
                    cal_data = yaml.load(cal_file, Loader=yaml.FullLoader)

                self.motor_id2init_pos = np.array(cal_data["motor_init_pos"])
                # This is called now that the self calibration has been done.
                # It can be called after the init_joints function
                self.mano_joints2spools_ratio = self.get_joints2spool_ratio()
                
                # Write zero angles for all joints 
                # TODO: Maybe put a different default position
                joint_angles = np.zeros(len(self.motor_ids)) 
                self.write_desired_joint_angles(joint_angles, calibrate=True)
            else:
                # Manual calibration code
                # Disable torque to allow the motors to move freely
                self.disable_torque()
                input("Move fingers to init position and press Enter to continue...")
                
                # Set to current based position control mode
                self.set_operating_mode(5)
                self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
                time.sleep(0.2)
                self.update_motorinitpos()


        #TODO: Maybe this need to change based on ratio kinematics model. Run and see what this norm value is. IF zero then just delete it.
        # self.motor_pos_norm = self.pose2motors(np.zeros(len(self.joint_ids)))


    def update_motorinitpos(self, motor_init_pos=None):
        """
        Updates the initial motor positions based on the current position
        """
        cal_yaml_fname = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "cal.yaml"
        )

        # Get current motor positions or the ones passed to the function
        if motor_init_pos is None:
            self.motor_init_pos = self.get_motor_pos()
        else:
            self.motor_init_pos = motor_init_pos
        print(
            f"Setting current motor position as motor_init_pos: {self.motor_init_pos}"
        )

        # Save the offsets to a YAML file
        cal_data = {}
        cal_data["motor_init_pos"] = self.motor_init_pos.tolist()
        with open(cal_yaml_fname, "w") as cal_file:
            yaml.dump(cal_data, cal_file, default_flow_style=False)

    def write_desired_joint_angles(self, joint_angles: np.array, calibrate: bool = False):
        """
        Command joint angles in deg
        :param: joint_angles: [joint 1 angle, joint 2 angle, ...]

        """

        # Pinky ABD mapped in reverse order
        joint_angles[14] *= -1

        # Thumb ABD and PIP mapped in reverse order
        joint_angles[1] *= -1
        joint_angles[2] *= -1 

        # Cheating way to make thumb MCP actuate more.
        joint_angles[3] += joint_angles[3] + (0.2*joint_angles[1] if joint_angles[1]>0 else 0)
 
        # Increase all joitns angles buy a little.
        # joint_angles[1:] *= 1.2

        joint_angles_clipped = self.clip_joint_angles(joint_angles)

        joint_angles_normalized = joint_angles_clipped - [low for low,_ in self.mano_joints_rom_list]
        
        motor_anlges_from_ratio = joint_angles_normalized * self.mano_joints2spools_ratio
        
        motor_pos_mapped = np.zeros(len(self.motor_ids))
        for i,idx in enumerate(self.mano_to_motor_ids_mapping):
            motor_pos_mapped[idx] = motor_anlges_from_ratio[i]

        # Abduction of Index and Middle finger are mapped in reverse order
        motor_pos_mapped[4] *=-1 # Index ABD
        motor_pos_mapped[7] *=-1 # Middle ABD
        motor_pos_mapped[-1] *=-1 # Wrist
        
        motor_pos_des = np.deg2rad(motor_pos_mapped) - self.motor_pos_norm + self.motor_id2init_pos

        if calibrate:
            # Move like this because the movement is big and joints will move too fast.
            # Creates overload error or breaks a tendon.
            self.move_to_desired_positions(motor_pos_des)
        else:
            self.write_desired_motor_pos(motor_pos_des)
            
        time.sleep(0.005) # wait for the command to be sent


    def get_mano_to_motor_ids_mapping(self):
        mano_indexes_list = []
        motors_ids_idxs_list = []
        for joint in self.mano_joint_mapping.keys():
            joint_motor_ids = self.motor_ids_dict[joint]
            joint_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in joint_motor_ids]      
            motors_ids_idxs_list.extend(joint_motor_idxs)

            joints_slice = self.mano_joint_mapping[joint]
            mano_indexes_list.extend(list(range(joints_slice.start, joints_slice.stop)))

        sorted_indices = sorted(range(len(mano_indexes_list)), key=lambda i: mano_indexes_list[i])
        mano_indexes_list = [mano_indexes_list[i] for i in sorted_indices]
        mapping = [motors_ids_idxs_list[i] for i in sorted_indices]
        return mapping

    def clip_joint_angles(self, joint_angles):
        """
        Clip the joint angles based on the ROM specified for each joint.
        :param joint_angles: Array of joint angles to be clipped.
        :return: Clipped joint angles.
        """
        # Get rom as array of 2 columns, one with min values and one with max values
        bounds = np.array(self.mano_joints_rom_list)

        # Separate bounds into min and max angle
        min_angles = bounds[:, 0]
        max_angle = bounds[:, 1]

        clipped_angles = np.clip(joint_angles, min_angles[:len(joint_angles)], max_angle[:len(joint_angles)])
        return clipped_angles

    def get_mano_joints_rom_list(self):
        """
        Get the ROM (Range of Motion) for each muscle group.
        :return: Dictionary with muscle group names as keys and tuples (lower ROM, upper ROM) as values.
        """
        joints_rom_list = [(0,0) for _ in range(17)]
        for muscle_group in self.muscle_groups:
            joint_indices = self.mano_joint_mapping[muscle_group.name]

            muscle_roms = muscle_group.joint_roms

            # idx is for geting the values from the muscle_roms which start from 0
            # joint_idx is for setting the values in the joints_rom_list which depends on the joint 
            for idx, joint_idx in enumerate(range(joint_indices.start,joint_indices.stop)):
                joints_rom_list[joint_idx] = (muscle_roms[idx][0], muscle_roms[idx][1])
        
        return joints_rom_list

    def get_joints2spool_ratio(self):
        """
        Get the ROM (Range of Motion) for each muscle group.
        :return: Dictionary with muscle group names as keys and tuples (lower ROM, upper ROM) as values.
        """
        joints_ratio_list = [0 for _ in range(17)]
        
        # calibration_ratios_file_name = self.find_latest_calibration_file("src/hand_control/calibration_yaml")
        calibration_ratios_file_name = "src/hand_control/calibration_yaml/calibration_ratios.yaml"
        
        # Open the YAML file
        if not os.path.isfile(calibration_ratios_file_name):
            raise FileNotFoundError(f"Calibration ratios file not found: {calibration_ratios_file_name}. \n Have you run the calibration script?")

        with open(calibration_ratios_file_name, "r") as yaml_file:
            calibration_defs = yaml.safe_load(yaml_file)
        
        for muscle_group in self.muscle_groups:
            finger_name = muscle_group.name
            
            joint_indices = self.mano_joint_mapping[finger_name]
            
            joint_names = list(calibration_defs[finger_name].keys())

            # idx is for geting the values from each ABD,MCP,PIP joint which start from 0
            # joint_idx is for setting the values in the joints_ratio_list which depends on the joint 

            for idx, joint_idx in enumerate(range(joint_indices.start,joint_indices.stop)):
                joint_type = joint_names[idx]
                if finger_name == "thumb":
                    joint_names_thumb = ["MCP","ABD","PIP","DIP"]
                    joint_type = joint_names_thumb[idx]
                elif finger_name == "wrist":
                    joint_type = "PITCH"
                    
                joints_ratio_list[joint_idx] = calibration_defs[finger_name][joint_type]["ratio"]

        return joints_ratio_list


if __name__ == "__main__" :
    gc = HandController("/dev/ttyUSB0", calibration= False, auto_calibrate= True)
    time.sleep(2.0)
