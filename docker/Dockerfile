# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
#FROM osrf/ros:kinetic-desktop-xenial
FROM usv_sim_dep

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]
 
RUN source /opt/ros/kinetic/setup.bash \
 && mkdir -p ~/catkin_ws/src \
 && cd ~/catkin_ws/src \
 # clone only if it was not clone yet
 #&& [ ! -d 'usv_sim_lsa' ] && git clone --recurse-submodules https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git 
 && rm -rf usv_sim_lsa && git clone --recurse-submodules https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git \
 # && if [ "$REVISION" != "$(dpkg-parsechangelog --show-field=Version)" ]; then exit 1; fi \
 && cd usv_sim_lsa  \
 && git submodule init && git submodule update \
 && chmod +x ./install_usv_sim \
 && rm -rf ~/.uwsim && ./install_usv_sim 
 
RUN source /opt/ros/kinetic/setup.bash \
 # && cd ~/catkin_ws/src/usv_sim_lsa \
 && cd ~/catkin_ws/ \
 && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y \
 && catkin_make_isolated --install \
 && source install_isolated/setup.bash

RUN echo 'source ~/catkin_ws/install_isolated/setup.bash' >> /root/.bashrc

# Set the working folder at startup
WORKDIR /root/catkin_ws
