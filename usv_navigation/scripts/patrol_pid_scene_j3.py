#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion
from std_srvs.srv import Empty
import rosbag
import subprocess
import os

waypoints = [
    [(-50.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-60.0, -5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-70.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-100.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]
speed = 
[
    [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
    [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0)],
    [(1.0, 0.0, 0.0), (0.0, 0.0, -1.0)],
    [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
]
result = Float64()
result.data = 0
x_offset = 400
y_offset = 120
maxSimulations = 1
maxTime = 8 * 60
f_distance = 2

def goal_vel(pose):
    goal_vel = Twist()
    goal_vel.linear = speed[i][0]
    goal_vel.angular = speed[i][1]
    return goal_vel

def get_result():
    global result, initial_pose, target_pose
    
    x1 = initial_pose.pose.pose.position.x
    y1 = initial_pose.pose.pose.position.y
    x2 = target_pose.pose.pose.position.x
    y2 = target_pose.pose.pose.position.y
    
    target_distance = math.hypot(x2-x1, y2-y1) 
    if target_distance < f_distance:
        return 1
    if target_distance >= f_distance:    
        return 0
    
def get_pose(initial_pose_tmp):
    global initial_pose 
    initial_pose = initial_pose_tmp

if __name__ == '__main__':
    global proc 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
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
                vel = goal_vel(pose)
                pub.publish(vel)
                rate.sleep()
        		if (rospy.get_time() > maxTime):
        			break;
                while get_result() == 0.0:
                    pub.publish(vel)
                    rate.sleep()
		    if (rospy.get_time() > maxTime):
			    break;
            simulationNumber = simulationNumber + 1
            if (simulationNumber > maxSimulations):
		        rospy.logerr("All simulations have been done. Pausing gazebo")
                pause() 
	        else:
                pause() 
		        rate.sleep()
                resetSimulation()
		        rate.sleep()
		        unpause()
		        rospy.logerr("Continue simulation!") 
        except rospy.ROSInterruptException:
	        rospy.logerr("ROS InterruptException! Just ignore the exception!") 
        except rospy.ROSTimeMovedBackwardsException:
	        rospy.logerr("ROS Time Backwards! Just ignore the exception!")
	
