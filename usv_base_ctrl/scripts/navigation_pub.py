#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry

def navigation():
    pub = rospy.Publisher('usv_position_setpoint', Odometry, queue_size=10) # only create a rostopic, navigation system TODO
    rospy.init_node('navigation_publisher')
    rate = rospy.Rate(60) # 10h

    x = -20.0
    y = -20.0

    msg = Odometry()
   # msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "navigation"
    msg.pose.pose.position = Point(x, y, 0.)
 
    while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()
   

if __name__ == '__main__':
    try:
        navigation()
    except rospy.ROSInterruptException:
        pass
