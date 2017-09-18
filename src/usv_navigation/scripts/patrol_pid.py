#!/usr/bin/env python

import rospy
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

def goal_pose(pose):
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose.pose.position = Point(pose[0][0], pose[0][1], 0.)
    return goal_pose

def get_result(result_aux):
    global result
    result.data = result_aux.data

if __name__ == '__main__':
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("move_usv/result", Float64, get_result)

    while True:
        for pose in waypoints:
            goal = goal_pose(pose)
            pub.publish(goal)
            rate.sleep()
            while result.data == 0.0:
                pub.publish(goal)
                rate.sleep()
