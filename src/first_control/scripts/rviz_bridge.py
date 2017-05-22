#!/usr/bin/env python

import roslib
import rospy 
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

'''
Simple bridge to transform thruster commands expressed as JointState to actual wrenches to be displayed in Rviz

'''

class Listener:
    def __init__(self):
        
        self.odom = Odometry()
        self.odom_received = False
        self.odom_sub = rospy.Subscriber('state', Odometry, self.OdomCallBack) 
        
        self.thruster_received = False
        self.thruster_sub = rospy.Subscriber('thruster_command', JointState, self.ThrusterCallBack)
            
    def OdomCallBack(self, msg): 
        self.odom_received = True
        self.odom = msg

    def ThrusterCallBack(self, msg): 
        self.thruster_received = True
        self.thruster = msg

if __name__ == '__main__':
    
    rospy.init_node('odom_to_tf')   
    
    br = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    
    wrench_pub = rospy.Publisher('thruster_wrench', WrenchStamped, queue_size=1)
            
    listener = Listener()
    
    # wait for thruster to initialize wrench dimension
    while not rospy.is_shutdown():   
        if listener.thruster_received:
            n_th = len(listener.thruster.name)
            break
        
    use_effort = (len(listener.thruster.name) == len(listener.thruster.effort))
        
    wrench_pub = [rospy.Publisher(name + '_wrench', WrenchStamped, queue_size=1) for name in listener.thruster.name]
    wrench = [WrenchStamped() for name in listener.thruster.name]
    for i,w in enumerate(wrench):
        w.header.frame_id = listener.thruster.name[i]

    T =1./50
    ratio = 1./5
    while not rospy.is_shutdown():        
        if listener.odom_received:            
            t = listener.odom.pose.pose.position
            q = listener.odom.pose.pose.orientation            
            
            br.sendTransform((t.x, t.y, t.z), (q.x,q.y,q.z,q.w), rospy.Time.now(), listener.odom.child_frame_id, listener.odom.header.frame_id)
            
            #br2.sendTransform((0,0,0), (q.x,q.y,q.z,q.w), rospy.Time.now(), listener.odom.child_frame_id, 'horiz')
            
        if listener.thruster_received:
            for i,w in enumerate(wrench):
                w.header.stamp = rospy.Time.now()
                if use_effort:
                    w.wrench.force.z = listener.thruster.effort[i]*ratio
                else:
                    w.wrench.force.z = listener.thruster.position[i]*ratio
                wrench_pub[i].publish(w)
            
            
        rospy.sleep(T)
