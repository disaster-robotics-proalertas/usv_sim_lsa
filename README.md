# usv_sim
water surface vehicles simulator;

1. cd ~/usv_sim_lsa
2. catkin_make_isolated
3. source ~/usv_sim_lsa/devel_isolated/setup.bash
4. launchs:
    4.1 for heading control: 
        roslaunch freefloating_gazebo_demo barco_ctrl_heading.launch
        publish the desired location on topic /barco_auv/usv_postion_setpoint
    4.2 for velocity control:
        roslaunch freefloating_gazebo_demo barco_ctrl_vel.launch
        publish the desired location on topic /barco_auv/cmd_vel
        

