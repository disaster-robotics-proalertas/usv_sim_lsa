#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion
from std_srvs.srv import Empty
import rosbag
import subprocess
import os
import time

waypoints = [
    [(10.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(0.0, 10.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]
result = Float64()
result.data = 0
x_offset = 240
y_offset = 95

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
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(10) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)

    while True:    
        for pose in waypoints:
            goal = goal_pose(pose)
            pub.publish(goal)
            rate.sleep()
            while result.data == 0.0:
                pub.publish(goal)
#                time.sleep(5)
                rate.sleep()
