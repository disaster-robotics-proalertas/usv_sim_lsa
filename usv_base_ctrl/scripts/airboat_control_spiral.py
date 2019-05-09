#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import tf

import atexit
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float64

initial_pose = Odometry()
target_pose = Odometry()
target_distance = 0
actuator_vel = 15
Ianterior = 0
rate_value = 50
Kp = 2
Ki = 0
result = Float64()
result.data = 0
f_distance = 3
rudder_min = -0.5
rudder_max = 0.5
rudder_med = (rudder_min + rudder_max)/2



startingTime=-1
startedToRun=False
indexArray = 0
# arrayIndex
# -1 waiting
# 0 executing PWM speeds of index array zero
# 1 executing PWM speeds of index array one

nothingPWM1 =0 # steering
nothingPWM2 =0 # propulsor
minPWM1 = -0.5	# steering
maxPWM1 = 0.5
minPWM2 = -4.12	# propulsor
maxPWM2 = 4.12

speedArray =[
	#leftPWM, rightPWM, time(seconds)
	(0.95*maxPWM1,   maxPWM2,0 ),
	(0.81*maxPWM1,   maxPWM2,12),
	(0.58*maxPWM1,   maxPWM2*0.82,24), 
	(-0.04*maxPWM1,  maxPWM2,42),
	(-0.62*maxPWM1,  maxPWM2*0.82,56),
	(-0.84*maxPWM1,  maxPWM2*0.86,72),
	(nothingPWM1, nothingPWM2,86)
]








rospy.init_node('zigzagTestNode')
rate = rospy.Rate(10)

pub_motor = rospy.Publisher('thruster_command', JointState, queue_size=10)
pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)



def thruster_ctrl_msg(value):
    global actuator_vel
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['fwd']
    msg.position = [value]
    msg.velocity = []
    msg.effort = []
    return msg

def rudder_ctrl_msg(value):
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['fwd_joint']
    msg.position = [value]
    msg.velocity = []
    msg.effort = []
    return msg

def exit_handler():
	global rate
	print 'Stop propeller'
	try:
            	pub_motor.publish(thruster_ctrl_msg(nothingPWM2))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	rate.sleep()
	try:
		pub_rudder.publish(rudder_ctrl_msg(nothingPWM1))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	rate.sleep()
	#print 'Trying to disarm!'
	#commandArming(False)


atexit.register(exit_handler)

def incrementIndexArray():
	global indexArray, commandPWM, speedArray
	indexArray = indexArray+1
	if (indexArray >= len(speedArray)):
		exit();
	try:
		print "indexArray: ",indexArray
		pub_motor.publish(thruster_ctrl_msg(speedArray[indexArray][1]))
		rate.sleep()
		pub_rudder.publish(rudder_ctrl_msg(speedArray[indexArray][0]))
		rate.sleep()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e





def spiralTest():
	global  commandPWM, startingTime, startedToRun
	
#	rospy.spin()
	while not rospy.is_shutdown():
		
		if (startedToRun==False):
				now = rospy.get_rostime()
				startingTime = now.secs+now.nsecs/1000000000.0
				startedToRun = True
		now = rospy.get_rostime()
		delta = now.secs + now.nsecs/1000000000 - startingTime
		print "delta: ",delta," > ",speedArray[indexArray][2]
		if (delta > speedArray[indexArray][2]):
			incrementIndexArray()

		rate.sleep()

if __name__== '__main__':
	try:
		spiralTest()
	except rospy.ROSInterruptException:
		print "Closing application"
		pass

