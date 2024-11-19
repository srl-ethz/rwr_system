# RWR System

Welcome to the RWR System repository. This repository contains the code and resources for the RWR class. Please note that there are several TODOs throughout the codebase that need to be addressed. Additionally, some configurations and settings may need to be adjusted to work with your specific setup.

**Disclaimer:** This repository is a work in progress and will be constantly updated and fixed. For any issues or questions, please use Slack to communicate or submit an issue on GitHub.

## Getting Started

To get started with the RWR System, follow these steps:

1. If you don't have a `src` directory in your ROS 2 workspace, create it:
    ```bash
    mkdir -p ~/ros2_ws/src
    ```
2. Clone the repository inside the `src` directory of your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/yourusername/rwr_system.git
    ```
3. Navigate to the project directory:
    ```bash
    cd rwr_system
    ```

    4. Activate your ROS 2 environment (assuming you are using mamba):
        ```bash
        mamba activate ros2 
        ```

        If you don't have a ROS 2 environment, follow this guide: [Getting Started with RoboStack](https://robostack.github.io/GettingStarted.html)

    5. Install the necessary dependencies:
        ```bash
        pip install -e .[all]
        ```
        this will install the faive_system package in your enviroment, and all the required dependencies specified in the `project.toml` file 


## Main TODOs for RWR students

- [ ] **Hand Control Package**: The `hand_control` package is the template provided in the Workshop on how to interface dynamixels. You have to adapt it to the specific kinematics of your fingers and hand. 
- [ ] **Retargeting File**: The `retargeter.py` file is tailored for P0, P1, or P4 hands. Adapt it for your hand model. You are free to develop a new retargeting method that better suits your needs. Refer to the workshop on 4/11 for guidance.
- [ ] **Visualization Files**: Update the visualization files to simulate your hand in Rviz. The `visualize_joints.py` script provides hints on visualizing a mixed rolling_contact/pin joints hand. You can also refer to the URDF and MJCF models of the P4 or P0 hands.
- [ ] **Logger**: The current logger is very basic. Feel free to enhance it to meet your requirements and make any necessary changes.



## ROS 2

To build the ROS 2 packages, follow these steps:

1. Navigate to your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws
    ```
2. Build the packages:
    ```bash
    colcon build --symlink-install
    ```
3. To build individual packages, use:
    ```bash
    colcon build --symlink-install --packages-up-to <package_name>
    ```
4. Source the setup script:
    ```bash
    source install/setup.bash
    ```

If you encounter any issues during the installation or build process, try removing the `install`, `log`, and `build` directories, and then attempt the process again:
```bash
rm -r install/ build/ log/
```
To create a new package, you can follow our examples or refer to this guide: [Creating Your First ROS 2 Package](https://docs.ros.org/en/eloquent/Tutorials/Creating-Your-First-ROS2-Package.html)


## Code Explanation

### Experiments

All experiments or main launch files are structured to ensure user interaction is straightforward. 

- To run scripts that require user interaction, use:
    ```bash
    ros2 run <package_name> <file_name>
    ```
    For example:
    ```bash
    ros2 run experiments run_teleop_rokoko.py
    ```

- To run all required nodes, use the corresponding launch files:
    ```bash
    ros2 launch <package_name> <launch_file>
    ```
    For example:
    ```bash
    ros2 launch experiments run_teleop_rokoko.launch.py
    ```
So, during a teleop experiment, you need to run the launch file in one terminal and the script in another terminal.

### Grasp GUI

The `grasp_gui.py` script is useful for testing your `hand_control`. Ensure you adapt it to your specific hand model.

### Logger

To start recording, use:
```bash
ros2 run logger logger_node.py
```
Select the list of topics to record in the code. Additionally, you can use the `record_demonstrations.launch.py` file to launch all the required nodes while recording.
### Other Packages

The project is organized into two main folders: one containing all the Python scripts and the other containing ROS-dependent components such as nodes, launch files, and packages.

#### Hand Control

Significant modifications are required for the hand control package. Refer to the workshop materials for guidance on controlling the hand.
We left the code in the package as reference or hint to implement your own kinematics and hand_controller. 

#### Ingress

##### OAK-D

This component handles the stream from the cameras.

- **OakdNode**: Publishes images to `/oakd/front_view/color`, etc.

##### Rokoko

This component handles the stream from the gloves.

- **RokokoNode**: Publishes data to `/ingress/mano` (21x3 keypoints, `FloatMultiArray`) and `/ingress/wrist` (`PoseStamped`).

#### Retargeter

 Your code should be placed in the `retargeter.py` file. Follow the provided hints to adapt it, or develop your own retargeting method.

- **RetargeterNode**: Subscribes to `/ingress/mano` and publishes to `/hand/policy_output` (you can rename this based on your hand model).

#### Viz
visualization helper functions, have a look at visualize_joints.py to see how to visualize both rolling contact and pin joints in rviz


### Example: Run Teleop and Collect Data

1. Run the launch file with all the nodes you want to run (e.g., `record_demonstration.launch.py`). You may need to modify this for your specific needs.
2. Run the teleop script:
    ```bash
    ros2 run experiments run_teleop_rokoko.py
    ```
    The calibration follows three steps:
    - The robot goes to the initial pose.
    - Calibrate the robot's initial pose once reached.
    - Mimic the robot pose with the glove, then calibrate the glove pose.
    - Now you have control of the robot (be careful!).
3. Run the logging script:
    ```bash
    ros2 run logger logger_node.py
    ```
    Prompt the correct task name.

## Contributing
If you would like to contribute to the RWR System, please fork the repository and submit a pull request. We welcome all contributions and improvements.

