#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    pub = rospy.Publisher('/barco_auv/thruster_command', JointState, queue_size=10)
    rospy.init_node('ctrl_jointState_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.name = ['fwd_left']
    hello_str.position = [10]
    hello_str.velocity = []
    hello_str.effort = []

    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
