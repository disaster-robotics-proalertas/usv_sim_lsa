# Simulated enviroment for Unmanned Surface Vehicles (usv_sim) -- 0.0.0
This simulator uses a combination of multiple physics packages to build a test enviroment for Unmanned Surface Vehicles (USV). We curently use UWsim for water surface modeling and ... . Our goal is to build a simulator for distater scenarios, such as floods, ... . We'll use it, at first, to develop and test control and trajectory strategies for USVs. but it can be easily adapted to other applications. It contains multiple robot models such as propeled boats and sailboats. You can find their xacros in packacge usv_sim.

## Getting Started

The main files to configure your simulations are:

1. XML files located into folder scenes of package usv_sim. In those files, you can configure UWSIM to run water simulation and some other world properties (current, waves and wind).
2. Launch files located into folder launch of package usv_sim. In those files, you can configure gazebo to run the simulation.
3. Xacro files located into folder xacro of package usv_sim. In those files, you can define the structure of your robot. You can use the available models (diferential boat, rudder boat, airboat and sailboat) as template to build your own model.

### Prerequisites

You need Ubuntu Linux 16.04 since the curent version of this simulator uses ROS Kinetic. To install Ros Kinetic, run the following commands:

        `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        `sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
        `sudo apt-get update`
        `sudo apt-get install ros-kinetic-desktop-full`
        `sudo rosdep init`
        `rosdep update`
        `sudo echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc`
        `source ~/.bashrc`
        `sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential`
        `sudo apt install python-rosdep python-wxtools`
        `sudo rosdep init`
        `rosdep update`
        `sudo apt-get install python-lxml python-pathlib python-h5py`
        `sudo apt-get install ros-kinetic-control-*`
        `sudo apt-get install ros-kinetic-osg-markers`
        `sudo apt-get install libfftw3-*`
        `sudo apt-get install libxml++2.6-*`
        

### Installing

1. cd ~
2. git clone https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git
3. cd ~/usv_sim_lsa
4. catkin_make_isolated --install
5. source ~/usv_sim_lsa/install_isolated/setup.bash

## Running the tests

6. launchs:

    6.1 for heading control:   
        `roslaunch freefloating_gazebo_demo barco_ctrl_heading.launch`  
        `publish the desired location on topic /barco_auv/usv_postion_setpoint`  

    6.2 for velocity control:    
        `roslaunch freefloating_gazebo_demo barco_ctrl_vel.launch`
        `publish the desired location on topic /barco_auv/cmd_vel`  

    6.3 for differential boat with user interface to define joints positions:  
        `roslaunch usv_sim boat_diff.launch parse:=true`  
        `roslaunch usv_sim boat_diff.launch parse:=false gui:=true`  

    6.4 for differential boat with heading control:  
        `roslaunch usv_sim boat_diff.launch parse:=true`  
        `roslaunch usv_sim boat_diff.launch parse:=false gui:=false`  

    6.5 for differential boat with user interface to define joints positions:  
        `roslaunch usv_sim boat_rudder.launch parse:=true`  
        `roslaunch usv_sim boat_rudder.launch parse:=false gui:=true`  

    6.6 for rudder boat with heading control:  
        `roslaunch usv_sim boat_rudder.launch parse:=true`  
        `roslaunch usv_sim boat_rudder.launch parse:=false gui:=false`  

    6.7 for sailboat with user interface to define joints positions:  
        `roslaunch usv_sim sailboat.launch parse:=true`  
        `roslaunch usv_sim sailboat.launch parse:=false gui:=true`  

    6.8 for sailboat with heading control:  
        `roslaunch usv_sim sailboat.launch parse:=true`  
        `roslaunch usv_sim sailboat.launch parse:=false gui:=false`  

    6.9 for airboat with user interface to define joints positions:  
        `roslaunch usv_sim airboat.launch parse:=true`  
        `roslaunch usv_sim airboat.launch parse:=false gui:=true`  

    6.10 for airboat with heading control:  
        `roslaunch usv_sim airboat.launch parse:=true`  
        `roslaunch usv_sim airboat.launch parse:=false gui:=false` 

On main folder of usv_sim_lsa, there are some scripts that run testing scenarios on Diluvio's River in Brazil. Each scenario is configured to test boat control on executing some common manueveurs (see image above). Besides that, you can run the following scripts:
	scenario1
	scenario2
	scenario3
	scenario4

To execute water simulation to those scenarios, you should run the script named ``waterCurrentDiluvio``.

<p align="center">
  <img src="./images/SCENARIOS2.png" width="800" alt="Scenarios to test boats"/>
</p>


## System Architecture 

The main system architecture is composed of UWSIM and Gazebo. With some plugins, we can simulate in a realistic way the effects of waves, wind and water currents on several boat types. Above is presented the some topic interaction between our gazebo plugin named usv_sailing_plugin and ROS Nodes usv_wind_current and usv_wind_current.

<p align="center">
  <img src="./images/DiagramaTopicosServicos.png" width="800" alt="System Architecture"/>
</p>

## Models


There are 4 boat models preconfigured in package usv_sim:
- airboat: composed by one thruster above the hull. This model has greater advantaged to navigate on shallow waters.
- differential boat: two thruster under water surface. This model has the simplest maneuverability.
- rudder boat: one thruster and one rudder. One of the most common configuration presented in boats.
- sailboat: one sail and one rudder.

<p align="center">
  <img src="./images/barcos4.png" width="800" alt="4 boat models"/>
</p>

The hull of all models above has been subdivided in 3 parts, so waves affects buoyancy of model in such way that boats present more realistic movement. If you want greater realism, you can subdivided the hull in more parts. To do that, you have to use geometric tools like Blender to model each part of hull. After that, you should configure links and joints in xacro files (like usv_sim/xacro/boat_common_subdivided). As gazebo simulator combine fixed joints, you should define the joints of hull as of type revolution, but with zero value to upper and lower limits. 

## Contributing

TODO

## Versioning

TODO

## Authors

* **Alexandre Amory**
* **Davi Henrique** 
* **Marcelo Paravisi** 

## License

TODO

## Acknowledgments

* freefloating_gazebo
* UWsim
* LiftDrag 
