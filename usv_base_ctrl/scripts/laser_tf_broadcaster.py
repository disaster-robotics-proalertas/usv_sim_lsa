#!/usr/bin/env python  
import roslib
import rospy
import tf
from sensor_msgs.msg import LaserScan

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    LaserScan laser_tf

if __name__ == '__main__':
    rospy.init_node('laser_tf_broadcaster')
    rospy.Subscriber('/laserScan', LaserScan, handle_turtle_pose)
    rospy.spin()
