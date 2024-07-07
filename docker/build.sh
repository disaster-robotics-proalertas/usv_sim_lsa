#!/bin/bash
# ATTENTION !!! run this script from a docker terminal !!!
source /opt/ros/kinetic/setup.bash
rm -rf ~/.uwsim/* && cp -r ~/catkin_ws/src/usv_sim_lsa/.uwsim/data ~/.uwsim/
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin_make_isolated --install
source install_isolated/setup.bash
