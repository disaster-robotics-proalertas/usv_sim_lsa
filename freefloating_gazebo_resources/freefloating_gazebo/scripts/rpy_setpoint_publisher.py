#!/usr/bin/env python

'''
A simple bridge to publish pose setpoint with RPY convention instead of quaternion
Useful for manually-given setpoints
To be run in the robot namespace with __ns:=XXX

Olivier Kermorgant
'''

import roslib
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler


class Listener:
    def __init__(self):
        
        self.setpoint = TwistStamped()
        self.setpoint_received = False
        self.setpoint_sub = rospy.Subscriber('body_rpy_setpoint', TwistStamped, self.SetpointCallBack) 
        
    def SetpointCallBack(self, msg): 
        self.setpoint_received = True
        self.setpoint = msg

if __name__ == '__main__':
    
    
    # rosnode
    rospy.init_node('rpy_setpoint_bridge')   
        
    pose_pub = rospy.Publisher('body_position_setpoint', PoseStamped, queue_size=1)
    pose_msg = PoseStamped()
            
    listener = Listener()

    T =1./50
    ratio = 1./5
    while not rospy.is_shutdown():        
        if listener.setpoint_received:
            # copy header
            pose_msg.header = listener.setpoint.header
            # just copy translation part
            pose_msg.pose.position = listener.setpoint.twist.linear
            # change rotation to quaternion
            q = quaternion_from_euler(listener.setpoint.twist.angular.x, listener.setpoint.twist.angular.y, listener.setpoint.twist.angular.z)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]            
            pose_pub.publish(pose_msg)            
            
        rospy.sleep(T)       
