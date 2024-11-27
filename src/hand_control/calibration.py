import os 
import yaml
import re
import numpy as np
import time
from datetime import datetime

# This is a class that contains the calibration functions for the hand_controller. 
# The hand_controller class should inherit from this class and call the functions in this class to calibrate the hand.
class CalibrationClass():

    def __init__(self):
        if type(self) is CalibrationClass:
            raise TypeError("CalibrationClass cannot be instantiated directly. It must be inherited.")
    
    def move_to_limit_and_get_pos(self, motor_start, calibration_current = 170, position_increment=0.1, threshold=0.0005):
        """
        Incrementally move motors to their limits based on the directions in motor_start.
        Stop when the change in position is below a threshold for all motors.

        :param motor_start: List of direction values for motors (positive/negative for direction, 0 to ignore).
        :param position_increment: Step size to increment or decrement the motor position.
        :param threshold: Minimum change in position to consider a motor as having reached its limit.
        :return: Final motor positions.
        """
        # Initialize the target positions for all motors
        motor_positions = self.get_motor_pos()
        target_positions = motor_positions.copy()

        self.write_desired_motor_current(calibration_current * np.ones(len(self.motor_ids)))

        while True:
            # Increment or decrement the target positions based on motor_start directions
            target_positions += position_increment * (motor_start > 0) - position_increment * (motor_start < 0)

            # Command all motors to move to the new target positions
            self.write_desired_motor_pos(target_positions)
            time.sleep(0.05)

            # Get the current motor positions
            current_positions = self.get_motor_pos()

            # Check if all motors have stopped moving significantly
            position_changes = abs(current_positions - motor_positions)
            all_motors_stopped = (position_changes <= threshold).all()
            motor_start[position_changes <= threshold] = 0
            # Update the motor positions for the next iteration
            motor_positions = current_positions
            if all_motors_stopped:
                break  # Exit the loop when all motors have reached their limits

        return motor_positions

    def auto_calibrate_fingers_with_pos(self, calib_current=120, maxCurrent: int = 150):
            """
            Calibrate each finger by extending the MCP joint fully in both directions and recording the motor positions.
            """
            motors_directions = np.ones(len(self.motor_ids))

            thumb_motor_ids = self.motor_ids_dict["thumb"]
            thumb_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in thumb_motor_ids]      
            index_motor_ids = self.motor_ids_dict["index"]
            index_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in index_motor_ids]    
            middle_motor_ids = self.motor_ids_dict["middle"]
            middle_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in middle_motor_ids]    
            ring_motor_ids = self.motor_ids_dict["ring"]
            ring_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in ring_motor_ids]    
            pinky_motor_ids = self.motor_ids_dict["pinky"]
            pinky_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in pinky_motor_ids]    
            
            motors_directions[middle_motor_idxs[0]] = -1
            motors_directions[index_motor_idxs[0]] = -1

            # Create a new calibration file
            date_created = datetime.now().strftime("%m-%d--%Hh")
            file_path = os.path.join("src/hand_control/calibration_yaml","calibration_"+ date_created+".yaml")        
            self.create_yaml_for_calibration([muscle_group.name for muscle_group in self.muscle_groups], file_path)
            
            # Open the YAML file
            with open(file_path, "r") as yaml_file:
                calibration_defs = yaml.safe_load(yaml_file)
            
            # Calibrate the wrist pitch joint
            # self.calibrate_wrist_pitch(file_path, calib_current, maxCurrent)
            
            wrist_motor_id = self.motor_ids_dict["wrist"][0]
            wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

            motor_pos_calib = np.ones(len(self.motor_ids))*calib_current
            motor_pos_calib[wrist_motor_idx] = 0

            motor_zero_current = np.zeros(len(self.motor_ids))
            motor_zero_current[wrist_motor_idx] = 0

            self.motor_id2init_pos = self.move_to_limit_and_get_pos(-1*motor_pos_calib*motors_directions, calibration_current=calib_current)

            # All motors should be fully extended in one direction at this point
            abd_pos_mean_list = []
            abd_motors_id_map_idx_list = []
            mcp_pos_mean_list = []
            mcp_motors_id_map_idx_list = []
            pip_pos_mean_list = []
            pip_motors_id_map_idx_list = []
            for muscle_group in self.muscle_groups:
                # For the moment we exclude thumb because probably different way to calibrate that
                abd_joint_index = 0  # Assuming MCP joint is the first joint in each muscle group
                mcp_joint_index = 1  # Assuming MCP joint is the second joint in each muscle group
                pip_joint_index = 2  # Assuming PIP joint is the third joint in each muscle group
                if muscle_group.name in ["index", "middle", "ring", "pinky", "thumb"]:

                    if muscle_group.name == "thumb":
                        abd_joint_index, mcp_joint_index = mcp_joint_index, abd_joint_index
                        dip_joint_index = 3  
                        # DIP
                        # Get the motor id for the DIP joint
                        dip_motor_id = muscle_group.motor_ids[dip_joint_index]
                        # Get the index of the motor id in the motor_ids list
                        dip_motors_id_map_idx = self.motor_ids.tolist().index(dip_motor_id)
                        dip_pos_extended = self.motor_id2init_pos[dip_motors_id_map_idx]

                    # ABD
                    # Get the motor id for the ABD joint
                    abd_motor_id = muscle_group.motor_ids[abd_joint_index]
                    # Get the index of the motor id in the motor_ids list
                    abd_motors_id_map_idx = self.motor_ids.tolist().index(abd_motor_id)
                    abd_motors_id_map_idx_list.append(abd_motors_id_map_idx)

                    # MCP
                    # Get the motor id for the MCP joint
                    mcp_motor_id = muscle_group.motor_ids[mcp_joint_index]
                    # Get the index of the motor id in the motor_ids list
                    mcp_motors_id_map_idx = self.motor_ids.tolist().index(mcp_motor_id)
                    mcp_motors_id_map_idx_list.append(mcp_motors_id_map_idx)

                    # PIP
                    # Get the motor id for the PIP joint
                    pip_motor_id = muscle_group.motor_ids[pip_joint_index]
                    # Get the index of the motor id in the motor_ids list
                    pip_motors_id_map_idx = self.motor_ids.tolist().index(pip_motor_id)
                    pip_motors_id_map_idx_list.append(pip_motors_id_map_idx)

                    # Get extended motor position for both MCP and PIP joint

                    abd_pos_extended = self.motor_id2init_pos[abd_motors_id_map_idx]
                    mcp_pos_extended = self.motor_id2init_pos[mcp_motors_id_map_idx]
                    pip_pos_extended = self.motor_id2init_pos[pip_motors_id_map_idx]

                    motor_pos = np.zeros(len(self.motor_ids))
                    motor_pos[mcp_motors_id_map_idx] = -calib_current
                    motor_pos[wrist_motor_idx] = 0

                    mcp_pos_flexed = self.move_to_limit_and_get_pos(-1*motor_pos*motors_directions, calibration_current=calib_current)[mcp_motors_id_map_idx]
                    mcp_pos_mean_list.append(np.mean([mcp_pos_extended, mcp_pos_flexed]))
                    mcp_pos_diff = np.rad2deg(np.abs(mcp_pos_extended - mcp_pos_flexed))

                    mcp_rom_range = muscle_group.joint_roms[mcp_joint_index][1] - muscle_group.joint_roms[mcp_joint_index][0]

                    motor_pos[abd_motors_id_map_idx] = -calib_current
                    motor_pos[mcp_motors_id_map_idx] = calib_current
                    motor_pos[pip_motors_id_map_idx] = -calib_current

                    if muscle_group.name == "thumb":
                        motor_pos[dip_motors_id_map_idx] = -calib_current

                    motor_pos[wrist_motor_idx] = 0

                    motor_pos_res = self.move_to_limit_and_get_pos(-1*motor_pos*motors_directions, calibration_current=calib_current)

                    # Get flexed ABD flexed motor position
                    abd_pos_flexed = motor_pos_res[abd_motors_id_map_idx]
                    abd_pos_diff = np.rad2deg(np.abs(abd_pos_extended-abd_pos_flexed))
                    # Get mean value in order to put the findger in the midle after calibration
                    abd_pos_mean_list.append(np.mean([abd_pos_extended, abd_pos_flexed]))
                    abd_rom_range = muscle_group.joint_roms[abd_joint_index][1] - muscle_group.joint_roms[abd_joint_index][0]

                    # Get flexed PIP flexed motor position
                    pip_pos_flexed = motor_pos_res[pip_motors_id_map_idx]
                    pip_pos_diff = np.rad2deg(np.abs(pip_pos_extended-pip_pos_flexed))
                    pip_pos_mean_list.append(np.mean([pip_pos_extended, pip_pos_flexed]))
                    pip_rom_range = muscle_group.joint_roms[pip_joint_index][1] - muscle_group.joint_roms[pip_joint_index][0]

                    # Save the end and start value of the motor position and save the ration
                    calibration_defs[muscle_group.name]["ABD"]["value"] = [float(abd_pos_flexed), float(abd_pos_extended)]
                    calibration_defs[muscle_group.name]["ABD"]["ratio"] = float(abd_pos_diff/abd_rom_range)

                    # Save the end and start value of the motor position and save the ration
                    calibration_defs[muscle_group.name]["MCP"]["value"] = [float(mcp_pos_flexed), float(mcp_pos_extended)]
                    calibration_defs[muscle_group.name]["MCP"]["ratio"] = float(mcp_pos_diff/mcp_rom_range)

                    # Save the end and start value of the motor position and save the ration
                    calibration_defs[muscle_group.name]["PIP"]["value"] = [float(pip_pos_flexed), float(pip_pos_extended)]
                    calibration_defs[muscle_group.name]["PIP"]["ratio"] = float(pip_pos_diff/pip_rom_range)

                    if muscle_group.name == "thumb":
                        dip_pos_flexed = motor_pos_res[dip_motors_id_map_idx]
                        dip_pos_diff = np.rad2deg(np.abs(dip_pos_extended-dip_pos_flexed))
                        dip_rom_range = muscle_group.joint_roms[dip_joint_index][1] - muscle_group.joint_roms[dip_joint_index][0]
                        calibration_defs[muscle_group.name]["DIP"]["value"] = [float(dip_pos_extended), float(dip_pos_extended)]
                        calibration_defs[muscle_group.name]["DIP"]["ratio"] = float(dip_pos_diff/dip_rom_range)


            # Write the structure to a YAML file
            with open(file_path, "w") as yaml_file:
                yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

            for i in range(len(abd_motors_id_map_idx_list)):
                self.motor_id2init_pos[abd_motors_id_map_idx_list[i]] = abd_pos_mean_list[i]

            for i in range(len(mcp_motors_id_map_idx_list)):
                self.motor_id2init_pos[mcp_motors_id_map_idx_list[i]] = mcp_pos_mean_list[i]

            for i in range(len(pip_motors_id_map_idx_list)):
                self.motor_id2init_pos[pip_motors_id_map_idx_list[i]] = pip_pos_mean_list[i]

            
            # Probably can delete this
            self.set_operating_mode(5)

            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            # Dummy code that moves the fingers mcp joints by 90 degrees.
            joint_angles = np.zeros(len(self.motor_ids))
            joint_angles[5] = 90
            joint_angles[8] = 90
            joint_angles[11] = 90
            joint_angles[14] = 90
            
            self.mano_joints2spools_ratio = self.get_joints2spool_ratio()

            self.write_desired_joint_angles(joint_angles)
            # self.write_desired_motor_pos(self.motor_id2init_pos)
            
            time.sleep(0.2)


    def calibrate_wrist_pitch(self, yaml_file_path, calib_current=40, maxCurrent: int = 150):
        """
        Calibrate the wrist joint by extending the pitch movement fully in both directions
        and recording the motor positions. Reads and writes results to the provided YAML file.
        
        Args:
            yaml_file_path (str): Path to the existing YAML calibration file.
            calib_current (int): Current value for calibration (default is 40).

        Returns:
            dict: Updated calibration results for the wrist pitch joint.
        """
        # Set to current control mode
        self.set_operating_mode(0)

        # Initialize motor position calibration array
        motor_pos_calib = np.zeros(len(self.motor_ids))

        # Get the motor ID for the wrist pitch joint
        
        wrist_motor_id = self.motor_ids_dict["wrist"][0]
        wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

        # Apply calibration current in one direction and record position
        motor_pos_calib[wrist_motor_idx] = calib_current
        wrist_pos_extended = self.write_current_and_get_pos(motor_pos_calib, np.zeros(len(self.motor_ids)))[wrist_motor_idx]

        # Apply calibration current in the opposite direction and record position
        motor_pos_calib[wrist_motor_idx] = -calib_current
        wrist_pos_flexed = self.write_current_and_get_pos(motor_pos_calib, np.zeros(len(self.motor_ids)))[wrist_motor_idx]

        # Calculate the range of motion (ROM) for the wrist joint
        wrist_pos_diff = np.rad2deg(np.abs(wrist_pos_extended - wrist_pos_flexed))
        
        wrist_min_rom, wrist_max_rop = self.mano_joints_rom_list[self.mano_joint_mapping["wrist"]][0] 
        wrist_rom_range = wrist_max_rop - wrist_min_rom

        # Calculate the mean position to center the wrist after calibration
        wrist_pos_mean = np.mean([wrist_pos_extended, wrist_pos_flexed])

        # Load the existing calibration definitions
        with open(yaml_file_path, "r") as yaml_file:
            calibration_defs = yaml.safe_load(yaml_file)

        # Save the start and end values, and the ratio for the wrist joint
        calibration_defs["wrist"] = {
            "PITCH": {
                "value": [float(wrist_pos_flexed), float(wrist_pos_extended)],
                "ratio": float(wrist_pos_diff / wrist_rom_range)
            }
        }

        # Write the updated calibration definitions back to the YAML file
        with open(yaml_file_path, "w") as yaml_file:
            yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

        # Set the motor position to the mean position for centering the wrist
        self.motor_id2init_pos[wrist_motor_idx] = wrist_pos_mean

        # Comment out later: Return to normal mode and set motor position
        self.set_operating_mode(5)
        self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))  # Max current for safety
        self.write_desired_motor_pos(self.motor_id2init_pos)
        time.sleep(0.2)


    def auto_calibrate_fingers(self, calib_current=40, maxCurrent: int = 150):
        """
        Calibrate each finger by extending the MCP joint fully in both directions and recording the motor positions.
        """
        calib_current = 140
        motors_directions = np.ones(len(self.motor_ids))

        thumb_motor_ids = self.motor_ids_dict["thumb"]
        thumb_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in thumb_motor_ids]      
        pinky_motor_ids = self.motor_ids_dict["pinky"]
        pinky_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in pinky_motor_ids]    

        motors_directions[thumb_motor_idxs[0]] = -1
        motors_directions[thumb_motor_idxs[1]] = -1
        motors_directions[thumb_motor_idxs[-1]] = -1
        motors_directions[pinky_motor_idxs[0]] = -1

        # Create a new calibration file
        date_created = datetime.now().strftime("%m-%d--%Hh")
        file_path = os.path.join("src/hand_control/calibration_yaml","calibration_"+ date_created+".yaml")        
        self.create_yaml_for_calibration([muscle_group.name for muscle_group in self.muscle_groups], file_path)
        
        # Open the YAML file
        with open(file_path, "r") as yaml_file:
            calibration_defs = yaml.safe_load(yaml_file)
        
        # Calibrate the wrist pitch joint
        # self.calibrate_wrist_pitch(file_path, calib_current, maxCurrent)
        
        wrist_motor_id = self.motor_ids_dict["wrist"][0]
        wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

        motor_ids_no_wrist = np.delete(self.motor_ids, wrist_motor_idx)

        # Set to current control mode to all motors except wrist
        self.set_operating_mode_for_motors(motor_ids_no_wrist,0)

        motor_pos_calib = np.ones(len(self.motor_ids))*calib_current
        motor_pos_calib[wrist_motor_idx] = calib_current

        motor_zero_current = np.zeros(len(self.motor_ids))
        motor_zero_current[wrist_motor_idx] = calib_current

        self.motor_id2init_pos = self.write_current_and_get_pos(motor_pos_calib*motors_directions, motor_zero_current) # Note: Wrist has been calibrated separately so the init_pos should be the same.

        # All motors should be fully extended in one direction at this point
        abd_pos_mean_list = []
        abd_motors_id_map_idx_list = []
        for muscle_group in self.muscle_groups:
            # For the moment we exclude thumb because probably different way to calibrate that
            abd_joint_index = 0  # Assuming MCP joint is the first joint in each muscle group
            mcp_joint_index = 1  # Assuming MCP joint is the second joint in each muscle group
            pip_joint_index = 2  # Assuming PIP joint is the third joint in each muscle group
            if muscle_group.name in ["index", "middle", "ring", "pinky", "thumb"]:

                if muscle_group.name == "thumb":
                    abd_joint_index, mcp_joint_index = mcp_joint_index, abd_joint_index
                    dip_joint_index = 3  
                    # DIP
                    # Get the motor id for the DIP joint
                    dip_motor_id = muscle_group.motor_ids[dip_joint_index]
                    # Get the index of the motor id in the motor_ids list
                    dip_motors_id_map_idx = self.motor_ids.tolist().index(dip_motor_id)
                    dip_pos_extended = self.motor_id2init_pos[dip_motors_id_map_idx]

                # ABD
                # Get the motor id for the ABD joint
                abd_motor_id = muscle_group.motor_ids[abd_joint_index]
                # Get the index of the motor id in the motor_ids list
                abd_motors_id_map_idx = self.motor_ids.tolist().index(abd_motor_id)
                abd_motors_id_map_idx_list.append(abd_motors_id_map_idx)

                # MCP
                # Get the motor id for the MCP joint
                mcp_motor_id = muscle_group.motor_ids[mcp_joint_index]
                # Get the index of the motor id in the motor_ids list
                mcp_motors_id_map_idx = self.motor_ids.tolist().index(mcp_motor_id)

                # PIP
                # Get the motor id for the PIP joint
                pip_motor_id = muscle_group.motor_ids[pip_joint_index]
                # Get the index of the motor id in the motor_ids list
                pip_motors_id_map_idx = self.motor_ids.tolist().index(pip_motor_id)

                # Get extended motor position for both MCP and PIP joint

                abd_pos_extended = self.motor_id2init_pos[abd_motors_id_map_idx]
                mcp_pos_extended = self.motor_id2init_pos[mcp_motors_id_map_idx]
                pip_pos_extended = self.motor_id2init_pos[pip_motors_id_map_idx]

                motor_pos = np.zeros(len(self.motor_ids))
                motor_pos[mcp_motors_id_map_idx] = -calib_current
                motor_pos[wrist_motor_idx] = calib_current

                mcp_pos_flexed = self.write_current_and_get_pos(motor_pos*motors_directions, motor_zero_current)[mcp_motors_id_map_idx]

                mcp_pos_diff = np.rad2deg(np.abs(mcp_pos_extended - mcp_pos_flexed))
                mcp_rom_range = muscle_group.joint_roms[mcp_joint_index][1] - muscle_group.joint_roms[mcp_joint_index][0]

                motor_pos[abd_motors_id_map_idx] = -calib_current
                motor_pos[mcp_motors_id_map_idx] = calib_current
                motor_pos[pip_motors_id_map_idx] = -calib_current

                if muscle_group.name == "thumb":
                    motor_pos[dip_motors_id_map_idx] = calib_current

                motor_pos[wrist_motor_idx] = calib_current

                motor_pos_res = self.write_current_and_get_pos(motor_pos*motors_directions, motor_zero_current)

                # Get flexed ABD flexed motor position
                abd_pos_flexed = motor_pos_res[abd_motors_id_map_idx]
                abd_pos_diff = np.rad2deg(np.abs(abd_pos_extended-abd_pos_flexed))
                # Get mean value in order to put the findger in the midle after calibration
                abd_pos_mean_list.append(np.mean([abd_pos_extended, abd_pos_flexed]))
                abd_rom_range = muscle_group.joint_roms[abd_joint_index][1] - muscle_group.joint_roms[abd_joint_index][0]

                # Get flexed PIP flexed motor position
                pip_pos_flexed = motor_pos_res[pip_motors_id_map_idx]
                pip_pos_diff = np.rad2deg(np.abs(pip_pos_extended-pip_pos_flexed))
                pip_rom_range = muscle_group.joint_roms[pip_joint_index][1] - muscle_group.joint_roms[pip_joint_index][0]

                # Save the end and start value of the motor position and save the ration
                calibration_defs[muscle_group.name]["ABD"]["value"] = [float(abd_pos_flexed), float(abd_pos_extended)]
                calibration_defs[muscle_group.name]["ABD"]["ratio"] = float(abd_pos_diff/abd_rom_range)

                # Save the end and start value of the motor position and save the ration
                calibration_defs[muscle_group.name]["MCP"]["value"] = [float(mcp_pos_flexed), float(mcp_pos_extended)]
                calibration_defs[muscle_group.name]["MCP"]["ratio"] = float(mcp_pos_diff/mcp_rom_range)

                # Save the end and start value of the motor position and save the ration
                calibration_defs[muscle_group.name]["PIP"]["value"] = [float(pip_pos_flexed), float(pip_pos_extended)]
                calibration_defs[muscle_group.name]["PIP"]["ratio"] = float(pip_pos_diff/pip_rom_range)

                if muscle_group.name == "thumb":
                    dip_pos_flexed = motor_pos_res[dip_motors_id_map_idx]
                    dip_pos_diff = np.rad2deg(np.abs(dip_pos_extended-dip_pos_flexed))
                    dip_rom_range = muscle_group.joint_roms[dip_joint_index][1] - muscle_group.joint_roms[dip_joint_index][0]
                    calibration_defs[muscle_group.name]["DIP"]["value"] = [float(dip_pos_extended), float(dip_pos_extended)]
                    calibration_defs[muscle_group.name]["DIP"]["ratio"] = float(dip_pos_diff/dip_rom_range)
 
        # Write the structure to a YAML file
        with open(file_path, "w") as yaml_file:
            yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

        for i in range(len(abd_motors_id_map_idx_list)):
            self.motor_id2init_pos[abd_motors_id_map_idx_list[i]] = abd_pos_mean_list[i]

        # Comment out later
        self.set_operating_mode(5)
        self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
        self.write_desired_motor_pos(self.motor_id2init_pos)
        time.sleep(0.2)

    def write_current_and_get_pos(self, motor_start, motor_stop):
        """
        Send current command to the motor, wait, get the motor positions, wait and stop sending current to mootors.
        Return the motor positions.
        """
        # Apply a small current to all motors in the same direction to move them
        self.write_desired_motor_current(motor_start)
        # Wait for a short time to allow motors to reach the calibration position
        time.sleep(1.6)  # Adjust this time based on the motor speed and required movement
        motor_pos = self.get_motor_pos()
        time.sleep(0.1)  # Adjust this time based on the motor speed and required movement
        # Stop motor movement by setting current to zero
        self.write_desired_motor_current(motor_stop)
        return motor_pos
    

    def create_yaml_for_calibration(self, finger_names, file_path):
        # Define the common structure for each joint
        joints = ["ABD", "MCP", "PIP"]

        # Initialize the muscle_groups dictionary
        calibration_defs = {}

        # Build the structure using nested for loops
        for finger in finger_names:
            
            calibration_defs[finger] = {}
            
            joints_to_use = joints

            if finger == "thumb":
                joints_to_use = joints + ["DIP"] # Thumb has an additional joint
            elif finger == "wrist":
                joints_to_use = ["PITCH"] # Wrist has only one joint

            for joint in joints_to_use:
                calibration_defs[finger][joint] = {
                    "value": [0,0],
                    "ratio": 0
                }

        # Write the structure to a YAML file
        with open(file_path, "w") as yaml_file:
            yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

    def find_latest_calibration_file(self, folder_path):
        # Regular expression to match the naming scheme
        pattern = r"calibration_(\d{2})-(\d{2})--(\d{2})h\.yaml"
        latest_file = None
        latest_time = None

        # Search through the folder
        for file_name in os.listdir(folder_path):
            match = re.match(pattern, file_name)
            if match:
                # Extract month, day, and hour
                month, day, hour = map(int, match.groups())
                # Parse into a datetime object
                file_time = datetime(datetime.now().year, month, day, hour)
                
                # Update latest file if this file is more recent
                if latest_time is None or file_time > latest_time:
                    latest_time = file_time
                    latest_file = file_name

        if latest_file is None:
            raise FileNotFoundError("No calibration files found with the specified naming scheme.")
        
        print("Latest calibration file found:", latest_file)
        return os.path.join(folder_path, latest_file)