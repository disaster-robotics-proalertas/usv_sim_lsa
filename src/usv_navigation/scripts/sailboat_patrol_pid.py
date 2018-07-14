#!/usr/bin/env python

import rospy
from tacking import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

waypoints = [
    [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(22.0, 0.0, 0.0), (0.0, 0.0, -0.984047240305, 0.177907360295)]
]

currentPoint = Point()
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
tackAngle = 60
tackDistance = 0.3
isTacking = 0

def goal_pose(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose[0][0]+x_offset, pose[0][1]+y_offset, 0.)
    return goal_pose

def goal_pose_tack(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose.x + x_offset, pose.y + y_offset, 0.)
    return goal_pose

def get_result(result_aux):
    global result
    result.data = result_aux.data

def get_pose(pose_tmp):
    global currentPoint
    currentPoint = pose_tmp.pose.pose.position

def get_heeling(result_aux):
    global heeling
    heeling.data = result_aux.data

def get_spHeading(result_aux):
    global spHeading
    spHeading.data = result_aux.data

def adjustFrame(sensor):
    if sensor > 180:
        sensor -= 360
    if sensor < -180:
        sensor += 360
    return sensor


def checkTacking():
    global isTacking 
    global heeling
    global spHeading
    global nextWindDir
    nextWindDir = abs(adjustFrame(heeling.data - spHeading.data))
    rospy.loginfo("heeling = %f", heeling.data) 
    rospy.loginfo("spHeading = %f", spHeading.data) 
    rospy.loginfo("nextWindDir = %f", nextWindDir) 
    if nextWindDir < 30 and isTacking == 0: 
        isTacking = 1

if __name__ == '__main__':
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol_tacking')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)
    rospy.Subscriber("state", Odometry, get_pose)
    rospy.Subscriber("heeling", Float64, get_heeling)
    rospy.Subscriber("spHeading", Float64, get_spHeading)
    for i in range(1,5):
        rate.sleep()

    while True:
        for pose in waypoints:
            if isTacking:
                tackingPose = tackPoints(currentPoint, goal_pose(pose).pose.pose.position, tackAngle, tackDistance, heeling.data, spHeading.data) 
                rospy.loginfo(str(tackingPose))
                #print(tackingPose)
                for pose in tackingPose:
                    goal = goal_pose_tack(pose)
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
            checkTacking()
