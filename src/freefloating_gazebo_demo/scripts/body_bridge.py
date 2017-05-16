#!/usr/bin/env python

# modules
from sensor_msgs.msg import JointState
import rospy, roslib
import time
from pylab import pi
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

class Setpoint:
    '''
    Listens to body manual setpoint
    '''
    def __init__(self):
        self.received = False
        self.value = PoseStamped()
        
        # subscribe to manual setpoints
        rospy.Subscriber('dummy_body_setpoint', JointState, self.read_manual_setpoint)

    def read_manual_setpoint(self, msg):
        '''
        Read manual setpoint (JointState) and extract vehicle part
        '''
        self.received = True
        # store header
        self.value.header = msg.header
        # get position setpoint
        self.value.pose.position.x = msg.position[0]
        self.value.pose.position.y = msg.position[1]
        self.value.pose.position.z = msg.position[2]
        # get orientation setpoint
        quaternion = quaternion_from_euler(pi/2 , msg.position[3], msg.position[4])
        self.value.pose.orientation.x = quaternion[0]
        self.value.pose.orientation.y = quaternion[1]
        self.value.pose.orientation.z = quaternion[2]
        self.value.pose.orientation.w = quaternion[3]
        
if __name__ == '__main__':
    '''
    Simple script to transform joint states (manual interface) to body setpoints
    '''    
    
    roslib.load_manifest('freefloating_gazebo_demo')
    rospy.init_node('body_bridge')
    
    # prepare to publish to body setpoint topic
    setpoint_publisher = rospy.Publisher('/g500arm5e/body_position_setpoint', PoseStamped, queue_size=10)
    
    # init listener to manual setpoint    
    setpoint = Setpoint()
    
    # rate
    T = 1./100
    
    while not rospy.is_shutdown():
        
        if setpoint.received:
            setpoint_publisher.publish(setpoint.value)
        rospy.sleep(T)
    
    
