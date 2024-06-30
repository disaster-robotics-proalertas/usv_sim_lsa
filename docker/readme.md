

# Building the Docker Images

USV_SIM works with Ubuntu xenial (16.04) and ROS kinetic.
The base [kinetic docker](https://github.com/osrf/docker_images/tree/master/ros/kinetic/ubuntu/xenial) is already available. Thus, USV_SIM docker is built on top of `osrf/ros:kinetic-desktop-full-xenial`. 

## Building the docker image w usv_sim dependencies
The following command created the docker image with all USV_SIM depedencies:

```bash
docker build --network=host --rm -f Dockerfile_dep -t usv_sim_dep .
```

Which is used, in this next step, to build USV_SIM itself.

## Hardware aceleration support

In case you want to try hardware accelaration support, you need to build an additional docker image. 

For example, to create a docker to support OpenGL.

```bash
docker build --network=host --rm -f Dockerfile_<GRAPHIC_CARD> -t usv_sim_intel .
```
In <GRAPHIC_CARD>, you should inform your graphic card brand. The options available are:
* intel

`Dockerfile_intel` installs Intel drivers on top of `usv_sim_dep` image. A similar thing can be done with [NVIDIA](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration). **Contributions required to create a Docker for NVIDIA support !!!**


In case you decide to use the hardware accelarated image, **then replace `usv_sim_dep` by `usv_sim_<GRAPHIC_CARD>` in the following section**.

## Building usv_sim

Clone the usv_sim source code in the host computer. For instance at `$HOME/catkin_ws/src/usv_sim_lsa`.

```bash
mkdir -p $HOME/catkin_ws/src/
cd $HOME/catkin_ws/src/
git clone --recurse-submodules https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git
cd usv_sim_lsa
git submodule init
git submodule update
chmod +x ./install_usv_sim
```

Now, we use the previous image (`usv_sim_dep`) to build the binary. Note that the `-v` argument
maps the host folder `$HOME/catkin_ws` into a folder visible within the created docker image (`/root/catkin_ws/`).
This way, both the source code and the binary are acessible from the host. The second mounted volume holds uwsim data (e.g. meshes)
that might be required in some scenarios.

```bash
docker run -it --rm -v $HOME/catkin_ws:/root/catkin_ws/ -v $HOME/.uwsim:/root/.uwsim --net=host usv_sim_dep bash
```

Within the docker terminal, run the following script to build usv_sim:

```bash
cd /root/catkin_ws/src/usv_sim_lsa/docker
./build.sh
```

You should see an output like this:

```
...
[ 77%] Built target wind_current_generate_messages_cpp
[ 77%] Built target wind_current_generate_messages_eus
[100%] Built target freefloating_gazebo_fluid
Install the project...
-- Install configuration: ""
-- Installing: /root/catkin_ws/install_isolated/_setup_util.py
-- Installing: /root/catkin_ws/install_isolated/env.sh
-- Installing: /root/catkin_ws/install_isolated/setup.bash
-- Installing: /root/catkin_ws/install_isolated/local_setup.bash
-- Installing: /root/catkin_ws/install_isolated/setup.sh
-- Installing: /root/catkin_ws/install_isolated/local_setup.sh
-- Installing: /root/catkin_ws/install_isolated/setup.zsh
-- Installing: /root/catkin_ws/install_isolated/local_setup.zsh
-- Installing: /root/catkin_ws/install_isolated/.rosinstall
-- Installing: /root/catkin_ws/install_isolated/lib/pkgconfig/freefloating_gazebo.pc
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/cmake/freefloating_gazeboConfig.cmake
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/cmake/freefloating_gazeboConfig-version.cmake
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/package.xml
-- Installing: /root/catkin_ws/install_isolated/lib/freefloating_gazebo/pid_control
-- Set runtime path of "/root/catkin_ws/install_isolated/lib/freefloating_gazebo/pid_control" to ""
-- Installing: /root/catkin_ws/install_isolated/lib/libfreefloating_gazebo_control.so
-- Set runtime path of "/root/catkin_ws/install_isolated/lib/libfreefloating_gazebo_control.so" to ""
-- Installing: /root/catkin_ws/install_isolated/lib/libfreefloating_gazebo_fluid.so
-- Set runtime path of "/root/catkin_ws/install_isolated/lib/libfreefloating_gazebo_fluid.so" to ""
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/scripts
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/scripts/uwsim_scene_to_gazebo_spawner.py
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/scripts/gen_pid.py
-- Installing: /root/catkin_ws/install_isolated/share/freefloating_gazebo/scripts/rpy_setpoint_publisher.py
<== Finished processing package [21 of 21]: 'freefloating_gazebo'
```

And the expected generated files are like this:

```bash
root@ale:~/catkin_ws/src/usv_sim_lsa/docker# ll ~/catkin_ws/
total 28
drwxrwxr-x  6 1000 1000 4096 Jun 30 00:59 ./
drwx------  1 root root 4096 Jun 30 00:59 ../
-rw-r--r--  1 root root   98 Jun 30 00:58 .catkin_workspace
drwxr-xr-x 23 root root 4096 Jun 30 01:01 build_isolated/
drwxr-xr-x 19 root root 4096 Jun 30 01:01 devel_isolated/
drwxr-xr-x  5 root root 4096 Jun 30 01:01 install_isolated/
drwxrwxr-x  3 1000 1000 4096 Jun 30 00:43 src/
root@ale:~/catkin_ws/src/usv_sim_lsa/docker# ll ~/.uwsim/
total 12
drwxrwxr-x 3 1000 1000 4096 Jun 30 01:05 ./
drwx------ 1 root root 4096 Jun 30 00:59 ../
drwxr-xr-x 7 root root 4096 Jun 30 01:05 data/
```

# Running USV_SIM with Docker image for the first time

First, let's test DISPLAY configuration of the usv_sim image to run GUI apps:

```bash
export DISPLAY=:0.0
xhost +local:docker
docker run -it --rm --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim_dep bash -c "xcalc"
```
You can use any GUI application for this test. 

Next, it's time to test Gazebo and USV_SIM. First, we run docker with the required volumes mentioned before, 
plus `--name` argument, useful in the next steps to open multiple terminals into the same docker image.

```bash
docker run -it --rm --net=host --name=usv_sim_test \
    -v $HOME/catkin_ws:/root/catkin_ws/ \
    -v $HOME/.uwsim:/root/.uwsim \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim_dep bash
```
Within the docker terminal, run: 

```bash
source ~/catkin_ws/install_isolated/setup.bash
roslaunch gazebo_ros empty_world.launch 
```

Finally, still in the same docker terminal used before, let's test a USV_SIM basic scenario:

```bash
roslaunch usv_sim airboat_scenario1.launch parse:=true
roslaunch usv_sim airboat_scenario1.launch parse:=false
```

The 1st command shows some red messages. It's normal. The 2nd command launchs an empty Gazebo window and the UWSIM window with the following scenario:

<p align="center">
  <img src="./scenario.png" width="400" alt="Scenarios to test boats"/>
</p>


# Interacting with the simulation

Assuming the previous docker image and the simulation scenario are still opened, open a second docker terminal attached to the same image, name `usv_sim_test`. 
This terminal will be used to launch RViz to send simulation commands:

```bash
docker docker exec -it usv_sim_test bash
```
Within the docker terminal, run: 

```bash
source ~/catkin_ws/install_isolated/setup.bash
roslaunch usv_sim_rviz navigation_rviz.launch
```

| :exclamation:  TBD: How to send the boat to a goal ?  |
|-------------------------------------------------------|

You can also inspect all boat topics to interact with it

```bash
root@ale:~/catkin_ws# rostopic list 
/airboat/Surface/fwd
/airboat/controllers/fwd_joint/position/parameter_descriptions
/airboat/controllers/fwd_joint/position/parameter_updates
/airboat/controllers/fwd_joint/velocity/parameter_descriptions
/airboat/controllers/fwd_joint/velocity/parameter_updates
/airboat/joint_command
/airboat/joint_setpoint
/airboat/joint_states
/airboat/move_usv/goal
/airboat/move_usv/result
/airboat/state
/airboat/thruster_command
/airboat/thruster_use
/clock
/gazebo/current
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/model/state
/rosout
/rosout_agg
/scan
/tf
/tf_static
/uwsim_marker/update
/uwsim_marker/update_full
```

# other scenarios to try

| :exclamation:  TBD |
|--------------------|

# References

Links to related tutorials and possible improvements:

 - https://github.com/Alok018/Jenkins/blob/master/Dockerfile
 - https://roboticseabass.com/2021/04/21/docker-and-ros/
 - https://github.com/rubensa/docker-ubuntu-tini-x11
 - https://www.howtogeek.com/devops/how-to-run-gui-applications-in-a-docker-container/
 - https://github.com/facontidavide/ros-docker-gazebo
 - http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
