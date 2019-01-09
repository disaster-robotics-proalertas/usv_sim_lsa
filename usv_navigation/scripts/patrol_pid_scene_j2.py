#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion
from std_srvs.srv import Empty
import time
import rosbag
import subprocess
import os

waypoints = [ #there is a offset applied to this  coordinates
    [(15.0, -5.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
    [(22.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(30.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(37.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(45.0, -5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(52.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]

waypoints = [ #there is a offset applied to this  coordinates
    [(-5.0, 15.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
    [( 0.0, 22.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [( 5.0, 30.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [( 0.0, 37.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-5.0, 45.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(0.0,  52.5, 0.0), (0.0, 0.0, 0.0, 1.0)]
]


result = Float64()
result.data = 0
x_offset = 220
y_offset = 75
maxSimulations = 1
maxTime = 8 * 60

def goal_pose(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose[0][0]+x_offset, pose[0][1]+y_offset, 0.)
    return goal_pose

def get_result(result_aux):
    global result
    result.data = result_aux.data

if __name__ == '__main__':
    global proc 
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    #unpause()


    simulationNumber = 1
    while not rospy.is_shutdown():    
        try:
            rospy.logerr("Simulation number %d", simulationNumber)
            for pose in waypoints:
                goal = goal_pose(pose)
                pub.publish(goal)
                rate.sleep()
                if (rospy.get_time() > maxTime):
                    break;
                while result.data == 0.0:
                    pub.publish(goal)
                    rate.sleep()
                    if (rospy.get_time() > maxTime):
                        break;
            
            simulationNumber = simulationNumber + 1
            rospy.logerr("Increasing simulationNumber. now: %d", simulationNumber)
            if (simulationNumber > maxSimulations):
                rospy.logerr("All simulations have been done. Pausing gazebo")
                pause()
            else:
                rospy.logerr("preparing new simulation!")
                #rospy.logerr("pause simulation!")
                pause()
                #rospy.logerr("wait!")
                time.sleep(1)
                #rospy.logerr("reset simulation!")
                resetSimulation()
                #rospy.logerr("wait!")   
                time.sleep(1)
                #rospy.logerr("start new simulation!")
                unpause()
                rospy.logerr("Continue simulation!") 
        except rospy.ROSInterruptException:
            rospy.logerr("ROS InterruptException! Just ignore the exception!") 
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
    
