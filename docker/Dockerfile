# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
#FROM osrf/ros:kinetic-desktop-xenial
FROM usv_sim_dep

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]
 
# Create a Catkin workspace and clone TurtleBot3 repos
RUN source /opt/ros/kinetic/setup.bash \
 && mkdir -p ~/catkin_ws/src \
 && cd ~/catkin_ws/src \
 && catkin_init_workspace \
 && git clone https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git  \
 && cd usv_sim_lsa  \
 && git submodule init  \
 && git submodule update  
 
RUN source /opt/ros/kinetic/setup.bash \
 && cd ~/catkin_ws/src/usv_sim_lsa \
 && chmod +x ./install_usv_sim \
 && ./install_usv_sim
 
RUN source /opt/ros/kinetic/setup.bash \
 && cd ~/catkin_ws/src/usv_sim_lsa \
 #&& rosdep install --from-paths src --ignore-src --rosdistro kinetic -y \
 && cd ~/catkin_ws/ \
 && catkin_make_isolated --install \
 && source install_isolated/setup.bash

# Set the working folder at startup
WORKDIR /catkin_ws
