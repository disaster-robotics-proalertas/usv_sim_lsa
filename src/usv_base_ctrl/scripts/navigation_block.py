#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

def navigation():
    rospy.Publisher('usv_position_setpoint', Odometry) # only create a rostopic, navigation system TODO
    rospy.spin()

   # while not rospy.is_shutdown():
   #     rospy.loginfo(hello_str)
   #     pub.publish(hello_str)
   #     rate.sleep()
   

if __name__ == '__main__':
    try:
        navigation()
    except rospy.ROSInterruptException:
        pass
