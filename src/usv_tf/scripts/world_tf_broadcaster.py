#!/usr/bin/env python  
import roslib
import rospy
import tf
import sys
from nav_msgs.msg import Odometry

def handle_turtle_pose(msg, br):
    # r = rospy.Rate(102)
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), rospy.Time.now(), sys.argv[1] + "_tf/base_link", sys.argv[1] + "_tf/odom")
    # r.sleep()

if __name__ == '__main__':
    rospy.init_node(sys.argv[1] + "world_tf_broadcaster")
    br = tf.TransformBroadcaster()
    rospy.Subscriber("state", Odometry, handle_turtle_pose, br)
    rospy.spin()
