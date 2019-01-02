# Simulated enviroment for Unmanned Surface Vehicles (usv_sim_lsa) -- 0.2

[![Build Status](https://travis-ci.org/disaster-robotics-proalertas/usv_sim_lsa.svg?branch=develop%2Ftravis_integration)](https://travis-ci.org/disaster-robotics-proalertas/usv_sim_lsa)
[![Read the Docs](https://readthedocs.org/projects/gazebo-usv-simulation/badge/?version=latest)](http://gazebo-usv-simulation.rtfd.io/)
[![Gitter](https://img.shields.io/gitter/room/nwjs/nw.js.svg)](https://gitter.im/usv-sim)
[![DOI](https://zenodo.org/badge/91500138.svg)](https://zenodo.org/badge/latestdoi/91500138)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/disaster-robotics-proalertas/usv_sim_lsa/blob/master/LICENSE)

This simulator uses a combination of multiple physics packages to build a test environment for Unmanned Surface Vehicles (USV).  We'll use it, at first, to develop and test control and trajectory strategies for USVs. but it can be easily adapted to other applications. It contains multiple robot models such as propeled boats(rudder boat, differential boat, airboat) and sailboat.
Boats are affected by waves, wind and water currents. To do that, we curently use UWsim for water surface modeling, we also load HEC-RAS output files with water speed of river and channel simulations. We simulate wind current with Lattice Boltzmann in a 2D grid. All those features alow to disturb the movement of boats in a realistic way.

## Prerequisites

You need Ubuntu Linux 16.04 since the curent version of this simulator uses ROS Kinetic. To install Ros Kinetic, run the following commands:


        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
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
        ./install_usv_sim

Compile the stack:

        cd ~/catkin_ws/
        catkin_make_isolated --install
        source install_isolated/setup.bash

To run a scenario:

        roslaunch usv_sim airboat_scenario1.launch parse:=true
        roslaunch usv_sim airboat_scenario1.launch parse:=false

The simulation might take some time to initialize if you're launching gazebo for the first time. If the simulation dosen't starts you should close it, run gazebo separately (command *gazebo* in the terminal), wait for gazebo to open (it is downloading some models), close gazebo and then try to run the scenario again.

Make sure your graphic card driver is up to date.

## Running the tests

On main folder of usv_sim_lsa, there are some scripts that run testing scenarios on Diluvio's River in Brazil. Each scenario is configured to test boat control on executing some common manueveurs (see image above). Besides that, you can run the following scripts:
- ``scenario1``: boat should navigate through two lines of buoys. 
- ``scenario2``: boat should avoid colision with 3 buoys.
- ``scenario3``: boat should execute zigzag to cover an area.
- ``scenario4``: boat should stay inside a circular area.

To execute water simulation to those scenarios, you should run the script named ``waterCurrentDiluvio``.

<p align="center">
  <img src="./images/SCENARIOS2.png" width="400" alt="Scenarios to test boats"/>
</p>

[//]: # ( AMA, aponta aqui para aqueles videos que vc fez antes. la no youtube, seria bom vc editar a descricao dos videos para incluir uma frase sobre o simulador e apontar p esse repo. assim, quem achar o teu simulador via youtube vai conseguir acessar o codigo )


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

The hull of all models above has been subdivided in 6 parts (see image above), so waves affects buoyancy of model in such way that boats present more realistic movement. If you want greater realism, you can subdivided the hull in more parts. To do that, you have to use geometric tools like Blender to model each part of hull. After that, you should configure links and joints in xacro files (like usv_sim/xacro/boat_subdivided4.xacro). As gazebo simulator combine fixed joints, you should define the joints of hull as of type revolution, but with zero value to upper and lower limits. 

<p align="center">
  <img src="./images/boatSubdivision3.png" width="800" alt="Boat subdivision"/>
</p>

## DISTURBANCE TYPES
The vehicles can be affected by 3 types of disturbances: wind currents, water currents and waves. Each kind of disturbance is presented below:

### WATER CURRENT
To allow the water current affect vehicles differently across the space and time, the USV_SIM can load output simulations from the HEC-RAS hydrological simulator. The HEC-RAS is a CFD software (computational fluid dynamics) capable of modelling the water flow through natural rivers and channels. Below it is presented an image, and a video of HEC-RAS simulations used in USV_SIM. HEC-RAS can reproduce turbulence effects presented into rivers and other bodies of water.

<p align="center">
  <img src="./images/composto4.png" width="800" alt="HEC-RAS Simulation of Diluvio River, Porto Alegre, RS - Brazil "/>
</p>

#### VIDEO - CLICK ON IMAGE TO PLAY IT
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=mbg59MQRa9M" target="_blank">
  <img src="./images/hecras_diluvio0.jpg" alt="Diluvio River - HEC-RAS Simulation" width="423" height="271" border="10" />
</a>
</p>

### WAVES
The integration with UWSim allows to reproduce waves to different configurations. Below is presented an differential boat travelling through waves.

[//]: # (This is also a comment. AMA. Marcelo, seria legal colocar um video mostrando bem de perto um barco balançando com as ondas.)


### WIND CURRENTS

The simulator can load wind currents generated by the CFD software OpenFoam, which it can solve continuum mechanics problems, and reproduce vortices and wind turbulence near buildings. Below it is presented some simulations that are avaliable in our robotic simulator. 

<p align="center">
  <img src="./images/bridge_1_5m_sem3D.png" width="400" alt="OpenFOAM simulation of bridge over Diluvio River, Porto Alegre, RS - Brazil "/>
  <img src="./images/slice_15m.png" width="400" alt="OpenFOAM simulation Porto Alegre, RS - Brazil"/>
</p>
<p align="center">
  <img src="./images/bridge0008.png" width="400" alt="OpenFOAM simulation of Porto Alegre, RS - Brazil"/>        
  <img src="./images/buildings_15m0008.png" width="400" alt="OpenFOAM simulation of Porto Alegre, RS - Brazil"/>        
</p>
P.S: The lines on the botton images have been integrated with runge kutta from wind current field.

#### VIDEO - CLICK ON IMAGE TO PLAY IT
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=i6lvMAsERHg" target="_blank">
  <img src="./images/openfoam_diluvio0.jpg" alt="Diluvio River - HEC-RAS Simulation" width="423" height="271" border="10" />
</a>
</p>

### HOW DISTURBANCES CAN AFFECT VEHICLES
The disturbances can affect vehicles in different ways:
- NONE: vehicle will not be affected by the disturbance;
- GLOBAL: vehicle will be affected by a global disturbance. It will not change in time and in space (same value to all over the place);
- LOCAL: vehicle will be affected by a local disturbance. It can change in time and in space. Disturbance values will acquired from wind_current (OpenFoam Simulation) and water_current (HEC-RAS Simulation);
#### CONFIGURATION

You can configure how the wind current and how the water current will affect each vehicle. Thus you can define on ''windType'' and ''waterType'' one of the following options: ''none'', ''global'', ''local''.

Below it is presented the portion of a launch file that it responsible to configure an airboat in the simulation. In this case, the ''windType'' and ''waterType'' was configurated with value ''local''. 

        <include file="$(find usv_sim)/launch/models/spawn_airboat_validation.launch">
                <arg name="gui" value="$(arg gui)"/>
                <arg name="spawnGazebo" value="$(arg spawnGazebo)"/>
                <arg name="namespace" value="$(arg namespace)"/>
                <arg name="windType" value="local"/>
                <arg name="waterType" value="local"/>
        </include>

Below, it is present another example, where a differential boat (named ''diffboat1'') was configurated in such way that the wind current has a global value and will not change in time, and the water current was defined to ''none'', so the vehicle will not be affected by wind currents.

        <include file="$(find usv_sim)/launch/models/spawn_diffboat_validation.launch">
                <arg name="gui" value="$(arg gui)"/>
                <arg name="spawnGazebo" value="$(arg spawnGazebo)"/>
                <arg name="namespace" value="diffboat1"/>
                <arg name="windType" value="global"/>
                <arg name="waterType" value="none"/>
        </include>

## GROUND TRUTH GENERATION
### CONFIGURATION
[//]: # ((This is also a comment. AMA. Marcelo, nao sei se eh necessario falaar sobre GROUND TRUTH GENERATION no git. mais eh instalacao, como rodar exemplo, como modificar o controle, como parametrizar a simulacao, como criar um scenario, etc. coisas de usabilidade.)

## Contributing

TODO

## Versioning

TODO

## Authors

* **Alexandre Amory**
* **Davi Henrique** 
* **Marcelo Paravisi** 
* **Vitor Augusto Machado Jorge**

## License

USV Simulator is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

## Acknowledgments

* freefloating_gazebo
* LiftDrag 
* UWsim - https://github.com/uji-ros-pkg/underwater_simulation
* Openfoam - https://openfoam.org/
* HEC-RAS - https://en.wikipedia.org/wiki/HEC-RAS 
* CAPES proalertas - https://lsa-pucrs.github.io/projects/pro-alertas
