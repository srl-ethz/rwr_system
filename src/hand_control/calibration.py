import os 
import yaml
import re
import numpy as np
import time
from datetime import datetime

# This is a class that contains the calibration functions for the hand_controller. 
# The hand_controller class should inherit from this class and call the functions in this class to calibrate the hand.
class CalibrationClass():

    def create_yaml_for_calibration(self, finger_names, file_path):
        # Define the common structure for each joint
        joints = ["ABD", "MCP", "PIP"]
        # Initialize the muscle_groups dictionary
        calibration_defs = {}

        # Build the structure using nested for loops
        for finger in finger_names:
            calibration_defs[finger] = {}
            for joint in joints:
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
        
        return os.path.join(folder_path, latest_file)

    def auto_calibrate_fingers(self, calib_current=40, maxCurrent: int = 150):
        """
        Calibrate each finger by extending the MCP joint fully in both directions and recording the motor positions.
        """
        # Set to current control mode
        self.set_operating_mode(0)

        calib_current = 100
        # motor_pos_calib = calib_current * np.ones(len(self.motor_ids))
        # motor_pos_calib[-1] = 0

        motor_pos_calib = np.zeros(len(self.motor_ids))
        motor_pos_calib[self.mano_joint_mapping["index"]] = calib_current
                
        self.motor_id2init_pos = self.write_current_and_get_pos(motor_pos_calib,np.zeros(len(self.motor_ids)))

        print("thumb abduction fully flexed is {}".format(self.motor_id2init_pos[4]))

        date_created = datetime.now().strftime("%m-%d--%Hh")
        file_path = os.path.join("src/hand_control/calibration_yaml","calibration_"+ date_created+".yaml")
        self.create_yaml_for_calibration([muscle_group.name for muscle_group in self.muscle_groups], file_path)
        
        # Open the YAML file
        with open(file_path, "r") as yaml_file:
            calibration_defs = yaml.safe_load(yaml_file)

        # All motors should be fully extended in one direction at this point

        # Set idexed corresponding to MCP and PIP joint
        abd_joint_index = 0  # Assuming MCP joint is the second joint in each muscle group
        mcp_joint_index = 1  # Assuming MCP joint is the second joint in each muscle group
        pip_joint_index = 2  # Assuming PIP joint is the third joint in each muscle group

        abd_pos_mean_list = []
        abd_motors_id_map_idx_list = []
        for muscle_group in self.muscle_groups:
            # For the moment we exclude thumb because probably different way to calibrate that
            # if muscle_group.name in ["index", "middle", "ring", "pinky"]:
            if muscle_group.name in ["index"]:

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

                mcp_pos_flexed = self.write_current_and_get_pos(motor_pos,np.zeros(len(self.motor_ids)))[mcp_motors_id_map_idx]

                mcp_pos_diff = np.rad2deg(np.abs(mcp_pos_extended - mcp_pos_flexed))
                mcp_rom_range = muscle_group.joint_roms[mcp_joint_index][1] - muscle_group.joint_roms[mcp_joint_index][0]

                motor_pos[abd_motors_id_map_idx] = -calib_current
                motor_pos[mcp_motors_id_map_idx] = calib_current
                motor_pos[pip_motors_id_map_idx] = -calib_current
                

                motor_pos_res = self.write_current_and_get_pos(motor_pos,np.zeros(len(self.motor_ids)))

                # Get flexed ABD flexed motor position
                abd_pos_flexed = motor_pos_res[abd_motors_id_map_idx]
                abd_pos_diff = np.rad2deg(np.abs(abd_pos_extended-abd_pos_flexed))
                # Get mean value in order to put the findger in the midle after calibration
                abd_pos_mean_list.append(np.mean([abd_pos_extended, abd_pos_flexed]))
                abd_rom_range = muscle_group.joint_roms[abd_joint_index][1] - muscle_group.joint_roms[abd_joint_index][0]
                print("thumb abduction fully extended is {}".format(abd_pos_extended))
                print("thumb abduction fully flexed is {}".format(abd_pos_flexed))
                print("thumb abduction mean is {}".format(np.mean([abd_pos_extended, abd_pos_flexed])))

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


    def self_calibrate(self, calib_current):
        """
        Calibrate the hand by moving the fingers to the initial position
        """
        # Set to current control mode
        self.set_operating_mode(0)

        # Apply a small current to all motors in the same direction to move them
        self.write_desired_motor_current(calib_current * np.ones(len(self.motor_ids)))

        # Wait for a short time to allow motors to reach the calibration position
        time.sleep(2)  # Adjust this time based on the motor speed and required movement

        # Stop motor movement by setting current to zero
        self.write_desired_motor_current(np.zeros(len(self.motor_ids)))

        # Set the current positions as the motor initialization positions
        self.update_motorinitpos()


    def write_current_and_get_pos(self, motor_start, motor_stop):
        """
        Send current command to the motor, wait, get the motor positions, wait and stop sending current to mootors.
        Return the motor positions.
        """
        # Apply a small current to all motors in the same direction to move them
        self.write_desired_motor_current(motor_start)

        # Wait for a short time to allow motors to reach the calibration position
        time.sleep(1.2)  # Adjust this time based on the motor speed and required movement
        motor_pos = self.get_motor_pos()
        time.sleep(0.2)  # Adjust this time based on the motor speed and required movement
        # Stop motor movement by setting current to zero
        self.write_desired_motor_current(motor_stop)
        return motor_pos