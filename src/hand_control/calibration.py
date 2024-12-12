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
    
    def move_to_desired_positions(self, desired_positions, position_increment=0.2, threshold=0.2):
        """
        Incrementally move motors to their desired positions.

        :param desired_positions: List of desired target positions for each motor.
        :param calibration_current: Current to apply to motors during movement.
        :param position_increment: Maximum step size to increment or decrement the motor position.
        :param threshold: Minimum change in position to consider a motor as having reached its target.
        :return: Final motor positions.
        """
        # Get the initial positions of all motors
        current_positions = self.get_motor_pos()

        # Run up to 10 seconds to reach the desired position. This is done because is oftens get stuch to a close 
        # enough position it cannot reach.
        start_time = time.time()
        while time.time() - start_time < 4:
            # Compute the difference between desired and current positions
            position_differences = desired_positions - current_positions

            # Calculate step adjustments for each motor
            step_adjustments = np.clip(position_differences, -position_increment, position_increment)

            # Calculate the new target positions
            new_positions = current_positions + step_adjustments

            # Command motors to move to the new target positions
            self.write_desired_motor_pos(new_positions)
            time.sleep(0.08)

            # Update the current positions
            current_positions = self.get_motor_pos()

            # Check if all motors are within the threshold of their desired positions
            if np.all(abs(desired_positions - current_positions) <= threshold):
                break

        return current_positions

    def move_to_limit_and_get_pos(self, motor_start, calibration_current = 180, position_increment=0.08, threshold=0.0002):
        """
        Incrementally move motors to their limits based on the directions in motor_start.
        Stop when the change in position is below a threshold get_motor_posor all motors.

        :param motor_start: List of direction values for motors (positive/negative for direction, 0 to ignore).
        :param position_increment: Step size to increment or decrement the motor position.
        :param threshold: Minimum change in position to consider a motor as having reached its limit.
        :return: Final motor positions.
        """
        # Initialize the target positions for all motors
        motor_positions = self.get_motor_pos()
        target_positions = motor_positions.copy()
        self.write_desired_motor_current(calibration_current * np.ones(len(self.motor_ids)))

        # while True:
        start_time = time.time()
        while time.time() - start_time < 5:
            # Increment or decrement the target positions based on motor_start directions
            target_positions += position_increment * (motor_start > 0) - position_increment * (motor_start < 0)
            
            # Command all motors to move to the new target positions
            self.write_desired_motor_pos(target_positions)
            time.sleep(0.08)

            # Get the current motor positions
            current_positions = self.get_motor_pos()

            # Check if all motors have stopped moving significantly
            position_changes = abs(current_positions - motor_positions)
            all_motors_stopped = (position_changes <= threshold).all()
            motor_start[position_changes <= threshold] = 0

            # Update the motor positions for the next iteration
            motor_positions = current_positions
            if all_motors_stopped:
                print("All motors stopped | It reached the limits")
                break  # Exit the loop when all motors have reached their limits
        
        if time.time() - start_time >= 10:
            print("Time limit reached")

        return self.get_motor_pos()

    def auto_calibrate_fingers_with_pos(self, calib_current=120, maxCurrent: int = 150):
            """
            Calibrate each finger by extending the MCP joint fully in both directions and recording the motor positions.
            """
            motors_directions = np.ones(len(self.motor_ids))

            # thumb_motor_ids = self.motor_ids_dict["thumb"]
            # thumb_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in thumb_motor_ids]      
            index_motor_ids = self.motor_ids_dict["index"]
            index_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in index_motor_ids]    
            middle_motor_ids = self.motor_ids_dict["middle"]
            middle_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in middle_motor_ids]    
            # ring_motor_ids = self.motor_ids_dict["ring"]
            # ring_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in ring_motor_ids]    
            # pinky_motor_ids = self.motor_ids_dict["pinky"]
            # pinky_motor_idxs = [self.motor_ids.tolist().index(motor_id) for motor_id in pinky_motor_ids]    
            
            # Move the index and middle finger ABD in the opposite direction
            motors_directions[middle_motor_idxs[0]] = -1
            motors_directions[index_motor_idxs[0]] = -1

            current_path = os.path.abspath(__file__)
            current_path = os.path.dirname(current_path)
            file_path = os.path.join(current_path,"calibration_yaml", "calibration_ratios.yaml")

            self.create_yaml_for_calibration([muscle_group.name for muscle_group in self.muscle_groups], file_path)
            
            # Calibrate the wrist pitch joint and keep it's initial position value.
            wrist_init_pos = self.calibrate_wrist_pitch_pos(file_path, calib_current, maxCurrent)
            
            # Open the YAML file
            with open(file_path, "r") as yaml_file:
                calibration_defs = yaml.safe_load(yaml_file)
            
            wrist_motor_id = self.motor_ids_dict["wrist"][0]
            wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

            motor_pos_calib = np.ones(len(self.motor_ids))*calib_current
            motor_pos_calib[wrist_motor_idx] = 0

            self.motor_id2init_pos = self.move_to_limit_and_get_pos(-1*motor_pos_calib*motors_directions, calibration_current=calib_current)
            
            # Give fully extended position of wrist found before
            self.motor_id2init_pos[wrist_motor_idx] = wrist_init_pos

            # This is the inital position of the hand. --> This is the .cal file
            self.update_motorinitpos(self.motor_id2init_pos)

            # All motors should be fully extended in one direction at this point
            # TODO: Delete these and there instances to clean up the code
            abd_pos_mean_list = []
            abd_motors_id_map_idx_list = []
            mcp_pos_mean_list = []
            mcp_motors_id_map_idx_list = []
            pip_pos_mean_list = []
            pip_motors_id_map_idx_list = []
            for muscle_group in self.muscle_groups:
                # For the moment we exclude thumb because probably different way to calibrate that
                abd_joint_index = 0  # Assuming ABD joint is the first joint in each muscle group
                mcp_joint_index = 1  # Assuming MCP joint is the second joint in each muscle group
                pip_joint_index = 2  # Assuming PIP joint is the third joint in each muscle group
                if muscle_group.name in ["index", "middle", "ring", "pinky", "thumb"]:

                    if muscle_group.name == "thumb":
                        # MCP is before ABD in thumb
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

                    # Get extended motor position for both ABD,MCP and PIP joint
                    abd_pos_extended = self.motor_id2init_pos[abd_motors_id_map_idx]
                    mcp_pos_extended = self.motor_id2init_pos[mcp_motors_id_map_idx]
                    pip_pos_extended = self.motor_id2init_pos[pip_motors_id_map_idx]

                    # Move the MCP joint to the opposite direction
                    motor_pos = np.zeros(len(self.motor_ids))
                    motor_pos[mcp_motors_id_map_idx] = -calib_current
                    motor_pos[wrist_motor_idx] = 0

                    mcp_pos_flexed = self.move_to_limit_and_get_pos(-1*motor_pos*motors_directions, calibration_current=calib_current)[mcp_motors_id_map_idx]
                    
                    mcp_pos_mean_list.append(np.mean([mcp_pos_extended, mcp_pos_flexed]))
                    mcp_pos_diff = np.rad2deg(np.abs(mcp_pos_extended - mcp_pos_flexed))

                    mcp_rom_range = muscle_group.joint_roms[mcp_joint_index][1] - muscle_group.joint_roms[mcp_joint_index][0]

                    # Move MCP back to fully extened position. 
                    # Flex the PIP joints and move the ABD joint to the opposite direction.
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
                    # calibration_defs[muscle_group.name]["ABD"]["ratio"] = float(abd_pos_diff/abd_rom_range)
                    calibration_defs[muscle_group.name]["ABD"]["ratio"] = float(np.ceil((abd_pos_diff/abd_rom_range)*10) / 10)

                    
                    # Save the end and start value of the motor position and save the ration
                    calibration_defs[muscle_group.name]["MCP"]["value"] = [float(mcp_pos_flexed), float(mcp_pos_extended)]
                    # calibration_defs[muscle_group.name]["MCP"]["ratio"] = float(mcp_pos_diff/mcp_rom_range)
                    calibration_defs[muscle_group.name]["MCP"]["ratio"] = float(np.ceil((mcp_pos_diff/mcp_rom_range)*10) / 10)


                    # Save the end and start value of the motor position and save the ration
                    calibration_defs[muscle_group.name]["PIP"]["value"] = [float(pip_pos_flexed), float(pip_pos_extended)]
                    # calibration_defs[muscle_group.name]["PIP"]["ratio"] = float(pip_pos_diff/pip_rom_range)
                    calibration_defs[muscle_group.name]["PIP"]["ratio"] = float(np.ceil((pip_pos_diff/pip_rom_range)*10) / 10)

                    if muscle_group.name == "thumb":
                        dip_pos_flexed = motor_pos_res[dip_motors_id_map_idx]
                        dip_pos_diff = np.rad2deg(np.abs(dip_pos_extended-dip_pos_flexed))
                        dip_rom_range = muscle_group.joint_roms[dip_joint_index][1] - muscle_group.joint_roms[dip_joint_index][0]
                        calibration_defs[muscle_group.name]["DIP"]["value"] = [float(dip_pos_flexed), float(dip_pos_extended)]
                        # calibration_defs[muscle_group.name]["DIP"]["ratio"] = float(dip_pos_diff/dip_rom_range)
                        calibration_defs[muscle_group.name]["DIP"]["ratio"] = float(np.ceil((dip_pos_diff/dip_rom_range)*10) / 10)


            # Write the structure to a YAML file
            with open(file_path, "w") as yaml_file:
                yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

            self.set_operating_mode(5)
            self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
            time.sleep(0.2)


    def calibrate_wrist_pitch_pos(self, yaml_file_path, calib_current=160, maxCurrent: int = 150):
        """
        Calibrate the wrist joint by extending the pitch movement fully in both directions
        and recording the motor positions. Reads and writes results to the provided YAML file.
        
        Args:
            yaml_file_path (str): Path to the existing YAML calibration file.
            calib_current (int): Current value for calibration (default is 40).

        Returns:
            dict: Updated calibration results for the wrist pitch joint.
        """
        # Initialize motor position calibration array
        motor_pos_calib = np.zeros(len(self.motor_ids))

        # Get the motor ID for the wrist pitch joint
        
        wrist_motor_id = self.motor_ids_dict["wrist"][0]
        wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

        # Apply calibration current in one direction and record position
        motor_pos_calib[wrist_motor_idx] = calib_current
        wrist_pos_extended = self.move_to_limit_and_get_pos(motor_pos_calib, calibration_current=calib_current)[wrist_motor_idx]

        # Apply calibration current in the opposite direction and record position
        motor_pos_calib[wrist_motor_idx] = -calib_current
        wrist_pos_flexed = self.move_to_limit_and_get_pos(motor_pos_calib, calibration_current=calib_current)[wrist_motor_idx]

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

        motor_pos_wrist = self.get_motor_pos()
        motor_pos_wrist[wrist_motor_idx] = wrist_pos_mean
        self.move_to_desired_positions(motor_pos_wrist)

        # Set the motor position to the mean position for centering the wrist
        time.sleep(0.2)
        return wrist_pos_extended

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
    
    def auto_calibrate_fingers_with_velocity(self, calib_current=120, maxCurrent: int = 150):
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
        # self.calibrate_wrist_pitch_pos(file_path, calib_current, maxCurrent)
        
        wrist_motor_id = self.motor_ids_dict["wrist"][0]
        wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

        motor_pos_calib = np.ones(len(self.motor_ids))*calib_current
        motor_pos_calib[wrist_motor_idx] = 0

        # motor_zero_current = np.zeros(len(self.motor_ids))
        # motor_zero_current[wrist_motor_idx] = 0

        wrist_motor_id = self.motor_ids_dict["wrist"][0]
        wrist_motor_idx = self.motor_ids.tolist().index(wrist_motor_id)

        motor_ids_no_wrist = np.delete(self.motor_ids, wrist_motor_idx)

        # Set to current control mode to all motors except wrist
        self.set_operating_mode_for_motors(motor_ids_no_wrist,1)

        self.motor_id2init_pos = self.move_to_limit_and_get_pos(-1*motor_pos_calib*motors_directions, calibration_current=calib_current)
        self.update_motorinitpos(self.motor_id2init_pos)

        print(self.motor_id2init_pos)
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
                # mcp_pos_diff = np.rad2deg(mcp_pos_extended - mcp_pos_flexed)

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
                # abd_pos_diff = np.rad2deg(abd_pos_extended-abd_pos_flexed)

                # Get mean value in order to put the findger in the midle after calibration
                abd_pos_mean_list.append(np.mean([abd_pos_extended, abd_pos_flexed]))
                abd_rom_range = muscle_group.joint_roms[abd_joint_index][1] - muscle_group.joint_roms[abd_joint_index][0]

                # Get flexed PIP flexed motor position
                pip_pos_flexed = motor_pos_res[pip_motors_id_map_idx]
                pip_pos_diff = np.rad2deg(np.abs(pip_pos_extended-pip_pos_flexed))
                # pip_pos_diff = np.rad2deg(pip_pos_extended-pip_pos_flexed)

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
                    # dip_pos_diff = np.rad2deg(dip_pos_extended-dip_pos_flexed)
                    dip_rom_range = muscle_group.joint_roms[dip_joint_index][1] - muscle_group.joint_roms[dip_joint_index][0]
                    calibration_defs[muscle_group.name]["DIP"]["value"] = [float(dip_pos_flexed), float(dip_pos_extended)]
                    calibration_defs[muscle_group.name]["DIP"]["ratio"] = float(dip_pos_diff/dip_rom_range)


        # Write the structure to a YAML file
        with open(file_path, "w") as yaml_file:
            yaml.dump(calibration_defs, yaml_file, default_flow_style=False)

        # Comment out later
        self.set_operating_mode(5)
        self.write_desired_motor_current(maxCurrent * np.ones(len(self.motor_ids)))
        time.sleep(0.2)

    def move_to_limit_with_velocity(self, motor_start, base_velocity=0.05, current_threshold=200, sleep_time=0.08):
        """
        Move motors to their limits using velocity control.
        Stop when the current exceeds a specified threshold for each motor.

        :param motor_start: List of direction values for motors (positive/negative for direction, 0 to ignore).
        :param base_velocity: Base velocity magnitude to apply (positive value).
        :param current_threshold: Current threshold in mA to determine when a motor has reached its limit.
        :param sleep_time: Time delay between updates in seconds.
        :return: Final motor positions.
        """
        # Initialize the velocity command for all motors
        motor_velocities = base_velocity * np.sign(motor_start)  # Apply direction based on motor_start

        # Continuously update velocities until limits are reached
        while True:
            # Command all motors to move with the specified velocities
            self.write_desired_motor_velocity(motor_velocities)
            time.sleep(sleep_time)

            # Get the current feedback for positions and currents
            current_positions = self.get_motor_pos()
            current_feedback = self.get_motor_current()

            # Check for motors exceeding the current threshold
            motors_at_limit = current_feedback >= current_threshold
            motor_start[motors_at_limit] = 0  # Stop motors that have reached the limit
            motor_velocities[motors_at_limit] = 0  # Set velocity to zero for those motors

            print(f"Current Positions: {current_positions}")
            print(f"Current Feedback: {current_feedback}")
            print(f"Motors at Limit: {motors_at_limit}")
            print("=" * 40)

            # If all motors have stopped, exit the loop
            if not any(motor_start):
                break

        # Stop all motors once the process is complete
        self.write_desired_motor_velocity(np.zeros(len(self.motor_ids)))

        # Return the final positions of all motors
        return self.get_motor_pos()