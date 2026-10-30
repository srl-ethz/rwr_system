
########## MODIFY THESE LINES #################

# Change to the location of your ros2 workspace
ros_workspace=~/workspace/ros2_ws/ 

# Change if your conda environment is named differently
conda activate ros2 

########## Do not modify below ###############

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source $ros_workspace/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$THIS_DIR/vision-allegro
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
