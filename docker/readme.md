

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

# Running USV_SIM with Docker image

xhost + 
 
docker run -it --rm --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    usv_sim_dep bash -it -c "xcalc"

# References


https://github.com/Alok018/Jenkins/blob/master/Dockerfile

https://roboticseabass.com/2021/04/21/docker-and-ros/
