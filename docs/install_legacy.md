## Legacy instalation procedure

You need Ubuntu Linux 16.04 since usv_sim uses ROS Kinetic. Since they are quite old software, we **HIGHLY recommend to follow the [docker-based installation](./install_docker.md)** procedure, which is also independent of the host setup you are currently used.

To install ROS Kinetic and some additional packages, run the following commands:


        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        sudo apt-get update
        sudo apt-get install ros-kinetic-desktop-full ros-kinetic-control-* ros-kinetic-osg-markers ros-kinetic-move-base -y
        sudo rosdep init
        rosdep update
        sudo echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc

Now run the following commands to download the dependencies of usv_sim:

        sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep python-wxtools python-lxml python-pathlib python-h5py python-scipy python-geolinks python-gdal -y
        sudo apt-get install libfftw3-* libxml++2.6-* libsdl-image1.2-dev libsdl-dev -y


## Installing

To run the packages of usv_sim you need a catkin workspace. If you already have a workspace you may jump to the Downloading and installing subsection.

### Creating a catkin workspace

        source /opt/ros/kinetic/setup.bash
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/
        catkin_make

### Downloading and installing usv_sim stack

Clone the usv_sim repository in the src folder of your catkin workspace:

        cd ~/catkin_ws/src
        git clone https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git
        cd usv_sim_lsa
        git submodule init
        git submodule update

Run the instalation script:

        cd ~/catkin_ws/src/usv_sim_lsa
        chmod +x ./install_usv_sim 
        ./install_usv_sim

Install the dependencies:

        rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

Compile the stack:

        cd ~/catkin_ws/
        catkin_make_isolated --install
        source install_isolated/setup.bash

To run a scenario:

        roslaunch usv_sim airboat_scenario1.launch parse:=true
        roslaunch usv_sim airboat_scenario1.launch parse:=false

The simulation might take some time to initialize if you're launching gazebo for the first time. If the simulation dosen't starts you should close it, run gazebo separately (command *gazebo* in the terminal), wait for gazebo to open (it is downloading some models), close gazebo and then try to run the scenario again.

Make sure your graphic card driver is up to date.