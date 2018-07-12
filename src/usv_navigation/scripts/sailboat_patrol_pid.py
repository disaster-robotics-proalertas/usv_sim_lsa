#!/usr/bin/env python

import rospy
from tacking import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

waypoints = [
    [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(5.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(7.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(10.0, -5.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(12.0, 0.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(14.0, 5.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(15.0, 0.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(16.0, -3.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(18.0, -5.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(22.0, 0.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)]
]

result = Float64()
result.data = 0
heeling = Float64()
heeling.data = 0
spHeading = Float64()
spHeading.data = 50
currentHeading = 0
windDirection = 0
x_offset = 240
y_offset = 95
nextWindDir = 100 

def goal_pose(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose[0][0]+x_offset, pose[0][1]+y_offset, 0.)
    return goal_pose

def get_result(result_aux):
    global result
    result.data = result_aux.data

def get_heeling(result_aux):
    global heeling
    heeling.data = result_aux.data

def get_spHeading(result_aux):
    global spHeading
    spHeading.data = result_aux.data

def checkTacking():
    global isTacking 
    global heeling
    global spHeading
    global nextWindDir
    nextWindDir = abs(heeling.data) - abs(spHeading.data)  
    rospy.loginfo("heeling = %f", heeling) 
    rospy.loginfo("spHeading = %f", spHeading) 
    rospy.loginfo("nextWindDir = %f", nextWindDir) 
    if nextWindDir < 30 and isTacking == 0: 
        isTacking = 1

if __name__ == '__main__':
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol_tacking')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)
    rospy.Subscriber("currentHeading"), Float64, get_current_heading)
    rospy.Subscriber("windDirection"), Float64, get_wind_heading)
    rospy.Subscriber("heeling"), Float64, get_heeling)
    rospy.Subscriber("spHeading"), Float64, get_spHeading)

    while True:
        for pose in waypoints:
            checkTacking()
            if isTacking:
                tacking = get tacking points
                for pose in tacking:
                    pub.publish(goal)
                    rate.sleep()
                    while result.data == 0.0:
                        pub.publish(goal)
                        rate.sleep()
            isTacking = 0
            goal = goal_pose(pose)
            pub.publish(goal)
            rate.sleep()
            while result.data == 0.0:
                pub.publish(goal)
                rate.sleep()
