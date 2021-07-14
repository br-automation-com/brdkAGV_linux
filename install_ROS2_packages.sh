#!/bin/bash
cd ros_ws/src/

# will either pull if the folder exist or clone it
 echo "SICK safety scanner base"
 [ -d "./sick_safetyscanners_base" ] && [ ! -L "./sick_safetyscanners_base/" ] && cd sick_safetyscanners_base/ && git pull && cd .. || git clone https://github.com/SICKAG/sick_safetyscanners_base
 echo "SICK safety scanner interfaces"
 [ -d "./sick_safetyscanners2_interfaces" ] && [ ! -L "./sick_safetyscanners2_interfaces/" ] && cd sick_safetyscanners2_interfaces/ && git pull && cd .. || git clone https://github.com/SICKAG/sick_safetyscanners2_interfaces
 
 
 cd ..
 source /opt/ros/galactic/setup.bash
 colcon build --symlink-install
 source /install/setup.sh
