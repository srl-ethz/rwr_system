cmake_minimum_required(VERSION 3.15)
project(retargeter)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Find system libraries
find_package(tf2_eigen REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files


#############
## Install ##
#############

install(PROGRAMS
    retargeter_node.py
    retargeter_collect_calibration_data.py
    retargeter_config_helper.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
