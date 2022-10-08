

# Building the Docker Images

USV_SIM works with Ubuntu xenial (16.04) and ROS kinetic.
The base [kinetic docker](https://github.com/osrf/docker_images/tree/master/ros/kinetic/ubuntu/xenial) is already available. Thus, USV_SIM docker is built on top of `osrf/ros:kinetic-desktop-full-xenial`. 

The following command created the docker image with all USV_SIM depedencies:

```
docker build --network=host --rm  -t usv_sim_dep .
```

Which is used, in this next step, to build USV_SIM it self


```
docker build --network=host --rm  -t usv_sim .
```

# Running USV_SIM with Docker image for the first time

First, let's test the usv_sim image in the simplest way. The following command will open a terminal where you can run commands without GUI: 

```
$ docker run -it --rm --net=host usv_sim bash
```

If this is working, next let's test the DISPLAY configuration to run GUI apps in the docker image:

```
$ xhost + 
$ xhost +local:docker
$ docker run -it --rm --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim bash -c "xcalc"
```
You can use any GUI application for this test. 

Next, it's time to test Gazebo, but without USV_SIM:

```
$ docker run -it --rm --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim bash -c "roslaunch gazebo_ros empty_world.launch"
```

Finally, let's test a USV_SIM basic scenario: 

```
$ docker run -it --rm --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    usv_sim bash -c "roslaunch usv_sim airboat_scenario1.launch parse:=true"
```

# References

 - https://github.com/Alok018/Jenkins/blob/master/Dockerfile
 - https://roboticseabass.com/2021/04/21/docker-and-ros/
