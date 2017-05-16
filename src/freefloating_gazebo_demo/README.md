freefloating_gazebo_demo
========================

This package contains two examples of underwater robot using the freefloating_gazebo plugin.

The first example uses UWSim for the rendering and Gazebo for the dynamics. Waypoints are given and followed by a simple PID controller.

1. Synchronize UWsim and Gazebo files (to do once)  
    This will scan the launchfile to create urdf from xacro files to be used in UWsim  
    `roslaunch freefloating_gazebo_demo g500arm5e.launch parse:=true`

2. Launch the simulators  
    This launches UWsim for the graphical part, and Gazebo to allow dynamic simulation.  
    Gazebo is launched with the freefloating_gazebo_fluid and freefloating_gazebo_control plugins.  
    A pid_control node is also launched and allows position and velocity control of the robot body and joints.  
    - UWsim: `roslaunch freefloating_gazebo_demo g500arm5e.launch`  
    - Gazebo: `roslaunch freefloating_gazebo_demo g500arm5e_gazebo.launch uwsim:=false`
    
3. Run the demo  
    This will unpause the physics (that are paused by default) and run the open-loop recovery of the black box.  
    Due to open-loop behavior the recovery will not always be performed.  
    `rosrun freefloating_gazebo_demo freefloating_gazebo_demo`
    
4. Manual control  
    The robot can also be controlled by hand using:  
    `roslaunch freefloating_gazebo_demo g500arm5e_manual.launch`  
    Two gui will appear, allowing to control the axes of the body and the joints of the arm.
    
The second example shows how to use steering thrusters and uses Gazebo only (rendering + dynamics):
	`roslaunch freefloating_gazebo_demo steering_thrusters.launch`
	Again, two gui will appear to control the thruste forces and two of the thruster orientation. 
