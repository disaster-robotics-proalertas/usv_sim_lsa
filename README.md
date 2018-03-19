# Simulated environment for Unmanned Surface Vehicles (usv_sim) -- 0.0.1
This simulator uses a combination of multiple physics packages to build a test environment for Unmanned Surface Vehicles (USV).  It is developed to test control and trajectory strategies for USVs, but it can be easily adapted to other applications. It contains multiple robot models such as propelled boats(rudder boat, differential boat, airboat) and sailboat. Boats are affected by waves, wind and water currents, implemented by UWsim for water surface modeling, HEC-RAS for water speed of river and channel simulations, and Lattice Boltzmann in a 2D grid for wind current. All those features allow to modelling the movement of boats in a realistic way.

### Prerequisites

You need Ubuntu Linux 16.04 since the current version of this simulator uses ROS Kinetic. To install ROS Kinetic, run the following commands:


        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
        sudo apt-get update
        sudo apt-get install ros-kinetic-desktop-full
        sudo rosdep init
        rosdep update
        sudo echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
        sudo apt install python-rosdep python-wxtools
        sudo apt-get install python-lxml python-pathlib python-h5py ros-kinetic-control-* ros-kinetic-osg-markers
        sudo apt-get install libfftw3-* libxml++2.6-* python-scipy python-geolinks python-gdal
        sudo apt-get instal ros-kinetic-move-base libsdl-image1.2-dev libsdl-dev
        sudo rosdep init
        rosdep update


        

### Installing

1. cd ~
2. git clone https://github.com/disaster-robotics-proalertas/usv_sim_lsa.git
3. cd ~/usv_sim_lsa
4. catkin_make_isolated --install
5. source ~/usv_sim_lsa/install_isolated/setup.bash

## Running the tests

On main folder of usv_sim_lsa, there are some scripts that run testing scenarios on Diluvio's River in Porto Alegre, Brazil. Each scenario is configured to test the boat control on executing some common maneuveurs (see image below). Besides that, you can run the following scripts:
- ``scenario1``: boat should navigate through two lines of buoys. 
- ``scenario2``: boat should avoid colision with 3 buoys.
- ``scenario3``: boat should execute zigzag to cover an area.

To execute water simulation to those scenarios, you should run the script named ``waterCurrentDiluvio``.

<p align="center">
  <img src="./images/SCENARIOS2.png" width="400" alt="Scenarios to test boats"/>
</p>

### VIDEOS

###### Scenario 1

<a href="http://www.youtube.com/watch?feature=player_embedded&v=2QfyyYbm6Zk" target="_blank">
 <img src="http://img.youtube.com/vi/2QfyyYbm6Zk/0.jpg" alt="Airboat - Scenario 1" width="290" height="210" border="10" />
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=9R6wtGL7XMU" target="_blank">
 <img src="http://img.youtube.com/vi/9R6wtGL7XMU/0.jpg" alt="Differential boat - Scenario 1" width="290" height="210" border="10" />
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=uduGP2FkSmU" target="_blank">
 <img src="http://img.youtube.com/vi/uduGP2FkSmU/0.jpg" alt="Rudder boat - Scenario 1" width="290" height="210" border="10" />
</a>

###### Scenario 2

<a href="http://www.youtube.com/watch?feature=player_embedded&v=eFy0dBdKnTg" target="_blank">
 <img src="http://img.youtube.com/vi/eFy0dBdKnTg/0.jpg" alt="Airboat - Scenario 2" width="290" height="210" border="10" />
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=Fx0n8Vdzoj8" target="_blank">
 <img src="http://img.youtube.com/vi/Fx0n8Vdzoj8/0.jpg" alt="Differential boat - Scenario 2" width="290" height="210" border="10" />
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=1V33dut5HRg" target="_blank">
 <img src="http://img.youtube.com/vi/1V33dut5HRg/0.jpg" alt="Rudder boat - Scenario 2" width="290" height="210" border="10" />
</a>

## System Architecture 

The main system architecture is composed of UWSIM and Gazebo, including plugins for realistic  waves, wind and water current simulation on several boat types. Above is presented the topic interaction between our gazebo plugin named usv_sailing_plugin and ROS Nodes usv_wind_current and usv_wind_current.

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

The hull of all models above has been subdivided in 6 parts (see image above), so waves affects buoyancy of model in such way that boats present more realistic movement. If you want greater realism, you can subdivided the hull in more parts. To do that, you have to use geometric tools like Blender to model each part of hull. After that, you should configure links and joints in xacro files (like usv_sim/xacro/boat_subdivided4.xacro). As gazebo simulator combine fixed joints, you should define the joints of hull as of type revolution, but with zero value to upper and lower limits. 

<p align="center">
  <img src="./images/boatSubdivision3.png" width="800" alt="Boat subdivision"/>
</p>




## LAUNCH FILES STRUCTURE

The main files to configure your simulations are:

1. XML files located into folder scenes of package usv_sim. In those files, you can configure UWSIM to run water simulation and some other world properties (current, waves and wind).
2. Launch files located into folder launch of package usv_sim. In those files, you can configure gazebo to run the simulation.
3. Xacro files located into folder xacro of package usv_sim. In those files, you can define the structure of your robot. You can use the available models (diferential boat, rudder boat, airboat and sailboat) as template to build your own model.

Each launch file was designed to include others files type in a way to customize your simulation. Above it is presented 3 tables that describe the relation between each launch file with xacro, xml, collada file (.dae).

|<sub> Launch File</sub> | <sub>Gazebo World File</sub> | <sub>UWSIM XML File</sub> | <sub>Autogenerated UWSIM File</sub> | <sub>Boat Launch File |
|-------------|-------------------|----------------|--------------------------|------------------|
|<sub>robots_start.launch</sub> | <sub>world/empty.world</sub> | <sub>robots_start.xml</sub> | <sub>robots_start_spawner.launch</sub> | <sub>models/spawn_boat_diff.launch<br> models/spawn_airboat.launch <br>models/spawn_boat_rudder.launch <br>models/spawn_sailboat.launch</sub> |
|<sub>scenario1.launch</sub> | <sub>world/empty.world</sub> | <sub>scenario1.xml</sub> | <sub>scenario1_spawner.launch</sub> | <sub>models/spawn_boat_diff.launch</sub> |
|<sub>scenario2.launch</sub> | <sub>world/empty.world</sub> | <sub>scenario2.xml</sub> | <sub>scenario2_spawner.launch</sub> | <sub>models/spawn_boat_diff.launch</sub> |
|<sub>scenario3.launch</sub> | <sub>world/empty.world</sub> | <sub>scenario3.xml</sub> | <sub>scenario3_spawner.launch</sub> | <sub>models/spawn_boat_diff.launch</sub> |

|<sub> Boat Launch File</sub> | <sub>Robot Description</sub> | <sub>Controller Configurations</sub> | <sub>PID Control</sub> | <sub>Velocity Control</sub> | <sub>TF Broadcasters</sub> | <sub>GUI Support</sub> | 
|-----------------|-------------------|-----------|----------------|------------------------------|-----------------|-------------|
|<sub>models/spawn_airboat.launch</sub></sub> | <sub>xacro/airboat.xacro</sub> | <sub>config/airboat.yaml</sub> | <sub>freefloating_gazebo::pid_control</sub> | <sub>usv_base_ctrl::control_vel_airboat.py</sub> | <sub>usv_tf::world_tf_broadcaster.py</sub> | <sub>Yes</sub> |
|<sub>models/spawn_boat_diff.launch</sub> | <sub>xacro/boat_diff.xacro</sub> | <sub>config/boat_diff.yaml</sub> | <sub>freefloating_gazebo::pid_control</sub> | <sub>usv_base_ctrl::control_diff_vel_ctrl.py</sub> | <sub>usv_tf::world_tf_broadcaster.py</sub> | <sub>Yes</sub> |
|<sub>models/spawn_boat_rudder.launch</sub> | <sub>xacro/boat_rudder.xacro</sub> | <sub>config/boat_rudder.yaml</sub> | <sub>freefloating_gazebo::pid_control</sub> | <sub>usv_base_ctrl::boat_rudder_vel_ctrl.py</sub> | <sub>usv_tf::world_tf_broadcaster.py</sub> | <sub>Yes</sub> |
|<sub>models/spawn_sailboat.launch</sub> | <sub>xacro/sailboat.xacro</sub> | <sub>config/sailboat.yaml</sub> | <sub>freefloating_gazebo::pid_control</sub> | <sub>usv_base_ctrl::control_??_ctrl.py</sub> | <sub>	usv_tf::world_tf_broadcaster.py</sub> | <sub>Yes</sub> | 


|<sub>Xacro Files</sub> |  <sub>Hull Subdivision</sub> |  <sub>Plugins</sub> |  <sub>DAE File</sub> |  <sub>Thruster</sub> |  <sub>Air thruster</sub> |  <sub>Rudder</sub> |
|------------|------------------|---------|-----------|---------|--------------|--------|
|<sub>xacro/airboat.xacro</sub> |  <sub>xacro/boat_subdivided4.xacro</sub> |  <sub>libfreefloating_gazebo_control</sub> |  |<sub> fwd </sub> |  | |  
|<sub>xacro/boat_diff.xacro</sub> |  <sub>xacro/boat_subdivided4.xacro</sub> |  <sub>libfreefloating_gazebo_control</sub> |  | <sub>fwd_left fwd_right</sub> |  | |
|<sub>xacro/boat_rudder.xacro</sub> | <sub>xacro/boat_subdivided4.xacro</sub> |  <sub>libfreefloating_gazebo_control libusv_sailing_plugin</sub> |  |<sub> fwd</sub> |  | <sub>rudder</sub> |
|<sub>xacro/sailboat.xacro</sub> |  <sub>xacro/boat_subdivided4.xacro</sub> |  <sub>libfreefloating_gazebo_control libusv_sailing_plugin</sub> |  <sub>meshes/simpleHull3/sail.dae meshes/simpleHull3/box.dae</sub> | | | |
|<sub>xacro/boat_subdivided4.xacro</sub> | - |<sub>libgazebo_ros_gpu_laser</sub> |  <sub>meshes/simpleHull3/base_link.dae meshes/simpleHull3/centerRight.dae meshes/simpleHull3/backLeft.dae meshes/simpleHull3/backRight.dae meshes/simpleHull3/frontLeft.dae meshes/simpleHull3/frontRight.dae meshes/simpleHull3/thruster.dae meshes/simpleHull3/airPropeller.dae meshes/simpleHull3/box.dae</sub> |  <sub>macro:thruster_link</sub> |  <sub>macro:airthruster_link</sub> |  <sub>macro:rudder_xacro</sub>|


## Contributing

One can contribute to this project by deteting bugs, future features, and pull requests. 

## Versioning

v0.0.1 – Initial version submitted to IROS 2018

## Authors

* Alexandre Amory (PUCRS University, Porto Alegre, Brazil)
* Davi Henrique (UFRN University, Natal, Brazil)
* Luiz Marcos Gonçalves (UFRN University, Natal, Brazil)
* Marcelo Paravisi (IFRS, Osorio, Brazil; PUCRS University, Porto Alegre, Brazil)
* Vitor Augusto Machado Jorge (PUCRS University, Porto Alegre, Brazil)
## License

TODO

## Acknowledgments

This project is supported by CAPES proalertas - https://lsa-pucrs.github.io/projects/pro-alertas
