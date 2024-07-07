# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
#FROM osrf/ros:kinetic-desktop-xenial
FROM osrf/ros:kinetic-desktop-full-xenial

# install ros packages

# setup sources.list
#RUN echo "deb http://snapshots.ros.org/kinetic/final/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-snapshots.list
# setup keys
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN echo "deb http://it.archive.ubuntu.com/ubuntu/ xenial main" >> /etc/apt/sources.list 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 9578539176BAFBC6
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#RUN apt-get update && apt-get install -y --no-install-recommends \
#    libsdl-dev \
#    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-control-* ros-kinetic-osg-markers ros-kinetic-move-base \
    python-rosinstall python-rosinstall-generator python-wstool build-essential \
    python-rosdep python-wxtools python-lxml python-pathlib python-h5py python-scipy python-geolinks python-gdal \
    libfftw3-* libxml++2.6-* libsdl-image1.2-dev libsdl-dev x11-apps \
    && rm -rf /var/lib/apt/lists/*

# install recommended tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git nano cmake cmake-curses-gui xauth \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
