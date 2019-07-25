#!/usr/bin/env python  
import roslib
import rospy
import tf
import sys
from nav_msgs.msg import MapMetaData

lastMsg = None

def vehicle_pose(msg):
    global lastMsg
    lastMsg = msg
    

if __name__ == '__main__':

    rospy.init_node("world_tf_broadcaster")
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/"+sys.argv[1]+"/map_metadata", MapMetaData, vehicle_pose)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (lastMsg != None):
            br.sendTransform((lastMsg.origin.position.x, lastMsg.origin.position.y, lastMsg.origin.position.z), (lastMsg.origin.orientation.x, lastMsg.origin.orientation.y, lastMsg.origin.orientation.z, lastMsg.origin.orientation.w), rospy.Time.now(), sys.argv[1]+"/odom", +sys.argv[1]+"/map")
        rate.sleep()
    
