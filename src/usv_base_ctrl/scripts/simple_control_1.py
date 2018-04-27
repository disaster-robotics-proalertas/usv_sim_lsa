#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

initial_pose = Odometry()
target_pose = Odometry()
target_distance = 0

def get_pose(initial_pose_tmp):
    global initial_pose 
    initial_pose = initial_pose_tmp

def get_target(target_pose_tmp):
    global target_pose 
    target_pose = target_pose_tmp

def actuator_ctrl():
    actuator_vel = 15
    return actuator_vel

def thruster_ctrl_msg():
    msg = JointState()
    msg.header = Header()
    msg.name = ['fwd_left']
    msg.position = [actuator_ctrl()]
    msg.velocity = []
    msg.effort = []
    return msg

def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor

def talker_ctrl():
    # publishes to thruster and rudder topics
    pub_motor = rospy.Publisher('/barco_auv/thruster_command', JointState, queue_size=10)
    pub_rudder = rospy.Publisher('/barco_auv/joint_setpoint', JointState, queue_size=10)
    rospy.init_node('usv_simple_ctrl', anonymous=True)
    rate = rospy.Rate(10) # 10h
    
    # subscribe to state and targer point topics
    rospy.Subscriber("/barco_auv/state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
    rospy.Subscriber("usv_position_setpoint", Odometry, get_target)  # get target position

    while not rospy.is_shutdown():
        pub_motor.publish(thruster_ctrl_msg())
        pub_rudder.publish(rudder_ctrl_msg())
        rate.sleep()

def rudder_ctrl():
    # erro = sp - atual
    # ver qual gira no horario ou anti-horario
    # aciona o motor (por enquanto valor fixo)
    global initial_pose
    global target_pose
    global target_distance

    x1 = initial_pose.pose.pose.position.x
    y1 = initial_pose.pose.pose.position.y
    x2 = target_pose.pose.pose.position.x
    y2 = target_pose.pose.pose.position.y

    # encontra angulo ate o ponto de destino (sp)
    myradians = math.atan2(y2-y1,x2-x1)
    sp_angle = math.degrees(myradians)

    target_distance = math.hypot(x2-x1, y2-y1) 

    # encontra angulo atual
    # initial_pose.pose.pose.orientation = nav_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))
    
    quaternion = (initial_pose.pose.pose.orientation.x, initial_pose.pose.pose.orientation.y, initial_pose.pose.pose.orientation.z,initial_pose.pose.pose.orientation.w) 
    
    euler = tf.transformations.euler_from_quaternion(quaternion) 

    # target_angle = initial_pose.pose.pose.orientation.yaw
    target_angle = math.degrees(euler[2])

    sp_angle = angle_saturation(sp_angle)
    target_angle = angle_saturation(target_angle)
    
    err = sp_angle - target_angle
    teste = 5

    log_msg = "sp: {0}; erro: {1}; x_atual: {2}; y_atual: {3}; x_destino: {4}; y_destino: {5}; teste: {6}" .format(sp_angle, err, initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y, target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, teste)

    rospy.loginfo(log_msg)

    rudder_angle = 90 * (-err/180)

    return rudder_angle

def rudder_ctrl_msg():
    msg = JointState()
    msg.header = Header()
    msg.name = ['rudder_joint']
    msg.position = [math.radians(rudder_ctrl())]
    msg.velocity = []
    msg.effort = []
    return msg

if __name__ == '__main__':
    try:
        talker_ctrl()
    except rospy.ROSInterruptException:
        pass

