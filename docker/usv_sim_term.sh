#!/bin/bash
# run this script from the host to lauch a usv_sim terminal
docker run -it --rm --net=host --name=usv_sim_test \
    -v $HOME/catkin_ws:/root/catkin_ws/ \
    -v $HOME/.uwsim:/root/.uwsim \
    -v $HOME/.gazebo:/root/.gazebo \
    -v $HOME/.ros:/root/.ros \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env="DISPLAY=$DISPLAY" \
    --env="XDG_SESSION_TYPE=x11" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim bash