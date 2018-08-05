#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import MapMetaData

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.origin.position.x, msg.origin.position.y, msg.origin.position.z), (msg.origin.orientation.x, msg.origin.orientation.y, msg.origin.orientation.z, msg.origin.orientation.w), rospy.Time.now(), "odom", "map")

if __name__ == '__main__':

    rospy.init_node("world_tf_broadcaster")
    rospy.Subscriber("map_metadata", MapMetaData, handle_turtle_pose)
    rospy.spin()
