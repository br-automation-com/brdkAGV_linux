# brdkAGV_linux

# Setup
This repository requires Ubuntu 20.04 with git installed

### Clone repository to home folder
    git clone https://github.com/br-automation-com/brdkAGV_linux.git
    cd brdkAGV_linux
    
### Run the ros2_installation.sh script
    sudo ./ros2_installation.sh

Guide can be found here: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

### Source the ros setup file
    source /opt/ros/galactic/setup.bash

### Build ROS workspace
    cd brdkAGV_linux/ros_ws
    colcon build
    
### Source the workspace setup file
    source install/local_setup.bash
    
### Now you can run a ros node for example
    ros2 run plc_comm opc_ua

## Documentation for ROS2 Galatic can be found here
https://docs.ros.org/en/galactic/
