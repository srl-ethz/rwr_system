# Faive System

## Getting Started

To install faive System, follow these steps:

1. If you don't have a `src` directory in your ROS 2 workspace, create it:
    ```bash
    mkdir -p ~/ros_ws/src
    ```
2. Clone the repository inside the `src` directory of your ROS 2 workspace:
    ```bash
    cd ~/ros_ws/src
    git clone https://github.com/srl_ethz/faive_system
    ```
3. Navigate to the project directory:
    ```bash
    cd faive_system
    ```
    1. Set up your ROS 2 Jazzy environment.
        If you don't have a ROS 2 environment, follow this guide: [Getting Started with RoboStack](https://robostack.github.io/GettingStarted.html).
        If you are using the pixi environment, you need to prepend all commands you want to run in the environment with `pixi run`, and run it from the `~/ros_ws` directory that you set up. Further commands in this README may abbreviate this prefixed command for brevity: re-add `pixi run` as needed.

    2. Install the necessary dependencies:
        For a pixi robostack environment, install pip first if it doesn't exist yet:
        ```bash
        pixi add pip
        # check that the correct pip is used:
        pixi run which pip
        ```
        The output of the `which` command should be `/home/your_usename/ros_ws/.pixi/envs/default/bin/pip` or similar. Note: Having the mamba / conda setup code in `~/.bashrc` or `~/.zshrc` might interfere with `pixi`'s `pip`, in which case comment out those setup scripts.

        Now install the dependent Python packages with
        ```
        pixi run pip install -e './src/faive_system[all]'
        ```
        this will install the faive_system package in your enviroment, and all the required dependencies specified in the `project.toml` file 
    
    3. Install other necessary dependencies
        ```bash
        pixi add ros-jazzy-rosidl-typesupport-c ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
        ```

4. Build ROS 2 packages
    1. Navigate to your ROS 2 workspace:
        ```bash
        cd ~/ros_ws
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
        Note: for pixi environments, add this to `~/ros_ws/pixi.toml` instead:
        ```toml
        [target.unix.activation]
        scripts = ["install/setup.bash"]
        ```

If you encounter any issues during the installation or build process, try removing the `install`, `log`, and `build` directories, and then attempt the process again:
```bash
rm -r install/ build/ log/
```


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
ros2 run logger logger_node.py # TODO: DEPRECATED
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

##### Apple Vision Pro (AVP)

This component handles the stream from the Apple Vision Pro headset.

- **AVPNode**: Publishes data to `/ingress/<chirality>/mano`, `/ingress/<chirality>/wrist` and other minor topics.

#### Retargeter

 Your code should be placed in the `retargeter.py` file. Follow the provided hints to adapt it, or develop your own retargeting method.

- **RetargeterNode**: Subscribes to `/ingress/mano` and publishes to `/hand/policy_output` (you can rename this based on your hand model).

#### Viz
visualization helper functions, have a look at visualize_joints.py to see how to visualize both rolling contact and pin joints in rviz


## Contributing
If you would like to contribute to the RWR System, please fork the repository and submit a pull request. We welcome all contributions and improvements.

