#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float64

initial_pose = Odometry()
target_pose = Odometry()
target_distance = 0
actuator_vel = 70
Ianterior = 0
rate_value = 10
Kp = 10
Ki = 0
left = 0
right = 0
result = Float64()
result.data = 0
#desired_distance = 3 # big scenarios
desired_distance = 2 # scenario G


def get_pose(initial_pose_tmp):
    global initial_pose 
    initial_pose = initial_pose_tmp

def get_target(target_pose_tmp):
    global target_pose 
    target_pose = target_pose_tmp

def thruster_ctrl_msg():
    global actuator_vel
    global left
    global right
    global Kp
    msg = JointState()
    msg.header = Header()
    msg.name = ['fwd_left', 'fwd_right']
    aux = rudder_ctrl()

    left = aux + 100.
    right = -aux + 100.
    if (left < 0):
        left = 0
    if (right < 0):
        right  = 0  

    msg.position = [left, right]
    msg.velocity = []
    msg.effort = []
    return msg

def verify_result():
    global target_distance
    global result
    if target_distance < desired_distance:
        result.data = 1
    if target_distance >= desired_distance:    
        result.data = 0
    return result

def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor

def sensor_saturation(sensor):
    if sensor > 100:
        sensor = 100
    if sensor < -100:
        sensor = -100
    return sensor

def talker_ctrl():
    global rate_value
    global result
    # publishes to thruster and rudder topics
    pub_motor = rospy.Publisher('thruster_command', JointState, queue_size=10)
    pub_result = rospy.Publisher('move_usv/result', Float64, queue_size=10)
    rospy.init_node('usv_simple_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value) # 10h
    
    # subscribe to state and targer point topics
    rospy.Subscriber("state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
    rospy.Subscriber("move_usv/goal", Odometry, get_target)  # get target position

    while not rospy.is_shutdown():
        try:    
            pub_motor.publish(thruster_ctrl_msg())
            pub_result.publish(verify_result())
            rate.sleep()
        except rospy.ROSInterruptException:
	    rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
	    rospy.logerr("ROS Time Backwards! Just ignore the exception!")

def P(erro):
    global Kp
    return Kp * erro

def I(erro):
    global Ki
    global Ianterior
    global rate_value
    if (Ianterior > 0 and erro < 0) or (Ianterior < 0 and erro > 0):
        Ianterior = Ianterior + Ki * erro * 50 * (1./rate_value)
    else:
        Ianterior = Ianterior + Ki * erro * (1./rate_value)
    return Ianterior

def rudder_ctrl():
    # erro = sp - atual
    # ver qual gira no horario ou anti-horario
    # aciona o motor (por enquanto valor fixo)
    global initial_pose
    global target_pose
    global target_distance
    global actuator_vel
    global Ianterior
    global rate_value
    global left
    global right

    x1 = initial_pose.pose.pose.position.x
    y1 = initial_pose.pose.pose.position.y
    x2 = target_pose.pose.pose.position.x
    y2 = target_pose.pose.pose.position.y

    # encontra angulo ate o ponto de destino (sp)
    myradians = math.atan2(y2-y1,x2-x1)
    sp_angle = math.degrees(myradians)

    target_distance = math.hypot(x2-x1, y2-y1) 
    actuator_vel = target_distance

    # encontra angulo atual
    # initial_pose.pose.pose.orientation = nav_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))
    
    quaternion = (initial_pose.pose.pose.orientation.x, initial_pose.pose.pose.orientation.y, initial_pose.pose.pose.orientation.z,initial_pose.pose.pose.orientation.w) 
    
    euler = tf.transformations.euler_from_quaternion(quaternion) 

    # target_angle = initial_pose.pose.pose.orientation.yaw
    target_angle = math.degrees(euler[2])

    sp_angle = angle_saturation(sp_angle)
    target_angle = angle_saturation(target_angle)
    
    err = sp_angle - target_angle
    err = angle_saturation(err)
    err = P(err) + I(err)

    if target_distance < desired_distance and x2 != 0 and y2 != 0:
        actuator_vel = 0
        err = 0

   # rudder_angle = 90 *(-err/180)
    rudder_angle = -err

    log_msg = "sp: {0}; actual_angle: {1}; erro: {2}; rudder_angle: {3}; left: {4}; right: {5}; target_distance: {6}; x_sp: {7}; y_sp: {8}".format(sp_angle, target_angle, err, rudder_angle, left, right, target_distance, target_pose.pose.pose.position.x, target_pose.pose.pose.position.y)
    #log_msg = "sp: {0}; erro: {1}; x_atual: {2}; y_atual: {3}; x_destino: {4}; y_destino: {5}; distancia_destino: {6}; rudder_angle: {7}" .format(sp_angle, err, initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y, target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_distance, rudder_angle)
    #rospy.loginfo(log_msg)
    return rudder_angle

#def rudder_ctrl_msg():
#    msg = JointState()
#    msg.header = Header()
#    msg.name = ['rudder_joint']
#    msg.position = [math.radians(rudder_ctrl())]
#    msg.velocity = []
#    msg.effort = []
#    return msg

if __name__ == '__main__':
    rospy.loginfo("Starting diff_control_heading")
    try:
        talker_ctrl()
    except rospy.ROSInterruptException:
	pass

