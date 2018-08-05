#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
import rosbag
import subprocess
import os
import math
import tf
import time

current_state = Odometry()
current_heading = 0 
max_vel = 0
max_vel_sail = 0
current_sail = -90
heading_range = 360 
sail_step = 5
heading_step = 5 
sim_time = 0.5 

def goal_pose1(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose[0][0]+x_offset, pose[0][1]+y_offset, 0.)
    return goal_pose

def get_pose(odom_aux):
    global current_state
    current_state = odom_aux

def rudder_ctrl_msg(sail_ctrl):
    msg = JointState()
    msg.header = Header()
    msg.name = ['rudder_joint', 'sail_joint']
    msg.position = [0, sail_ctrl]
    msg.velocity = []
    msg.effort = []
    return msg

def set_sailboat_heading(pub_state):
    global current_state
    global current_sail
    state_aux = ModelState()
    quaternion = (current_state.pose.pose.orientation.x, current_state.pose.pose.orientation.y, current_state.pose.pose.orientation.z, current_state.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], math.radians(current_heading))
    state_aux.pose.orientation.x = quaternion[0]
    state_aux.pose.orientation.y = quaternion[1]
    state_aux.pose.orientation.z = quaternion[2]
    state_aux.pose.orientation.w = quaternion[3]
    state_aux.model_name = 'sailboat'
    #state_aux.pose.position.x = current_state.pose.pose.position.x
    #state_aux.pose.position.y = current_state.pose.pose.position.y
    #state_aux.pose.position.z = current_state.pose.pose.position.z
    #print(current_state)

    state_aux.pose.position.x = 240 
    state_aux.pose.position.y = 95
    state_aux.pose.position.z = 1
    pub_state.publish(state_aux)
   
def talker():
    global current_state
    global current_heading
    global max_vel_sail
    global max_vel
    global current_sail
    global heading_range
    global sail_step
    global heading_step
    global sim_time
    rospy.init_node('polar_diagram')
    rate = rospy.Rate(10) # 10h
    rospy.Subscriber("/sailboat/state", Odometry, get_pose)
    pub_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    pub_sail = rospy.Publisher('/sailboat/joint_setpoint', JointState, queue_size=10)

    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    unpause()

    while current_heading <= heading_range:
        max_vel = 0
        current_sail = -90
        while current_sail <= 90:
            pub_sail.publish(rudder_ctrl_msg(math.radians(current_sail)))
            current_vel = 0
            start_time = time.time()
            elapsed_time = 0
            while elapsed_time < sim_time and current_state.twist.twist.linear.x > -0.2:
                current_vel = current_state.twist.twist.linear.x
                elapsed_time = time.time() - start_time

            if current_vel > max_vel:
                max_vel_sail = current_sail
                max_vel = current_vel

            print("Current Heading: ")
            print(current_heading)
            print("Current Sail Angle: ")
            print(current_sail)

            current_sail += sail_step 

            resetSimulation()
            #resetWorld()
            pause()
            set_sailboat_heading(pub_state)
            unpause()
            rate.sleep()

        x = rospy.get_param('/uwsim/wind/x')
        y = rospy.get_param('/uwsim/wind/y')
        wind_vel = math.sqrt(math.pow(x,2)+math.pow(y,2))
        polar = open("polar_diagram.txt","a")
        print_diagram = "%f,%f,%f,%f\n" % (max_vel, max_vel_sail, current_heading, wind_vel)
        print(print_diagram)
        polar.write(print_diagram)
        polar.close()

        resetSimulation()
        #resetWorld()
        #while rospy.get_time() > 1:
        #    show = 'show'
        pause()

        current_heading += heading_step 

        set_sailboat_heading(pub_state)

        unpause()
        rate.sleep()

        if current_heading >= heading_range:
            pause()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
