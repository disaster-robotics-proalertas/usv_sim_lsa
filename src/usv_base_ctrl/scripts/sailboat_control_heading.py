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
actuator_vel = 15
Ianterior = 0
rate_value = 10
Kp = 10 
Ki = 0 
result = Float64()
result.data = 0
sail_min = -math.pi/3
sail_max = math.pi/3
curent_heading = 0


def get_pose(initial_pose_tmp):
    global initial_pose 
    initial_pose = initial_pose_tmp

def get_target(target_pose_tmp):
    global target_pose 
    target_pose = target_pose_tmp

def sail_ctrl_msg():
    global actuator_vel
    msg = JointState()
    msg.header = Header()
    msg.name = ['fwd']
    msg.position = [sail_ctrl()]
    msg.velocity = []
    msg.effort = []
    return msg

def verify_result():
    global target_distance
    global result
    if target_distance < 5:
        result.data = 1
    if target_distance >= 5:    
        result.data = 0
    return result

def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor

def talker_ctrl():
    global rate_value
    rospy.init_node('usv_simple_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value) # 10h
    # publishes to thruster and rudder topics
    pub_sail = rospy.Publisher('angleLimits', Float64, queue_size=10)
    pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)
    pub_result = rospy.Publisher('move_usv/result', Float64, queue_size=10)
    
    # subscribe to state and targer point topics
    rospy.Subscriber("state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
    rospy.Subscriber("move_usv/goal", Odometry, get_target)  # get target position

    while not rospy.is_shutdown():
        pub_rudder.publish(rudder_ctrl_msg())
        pub_sail.publish(sail_ctrl())
        pub_result.publish(verify_result())
        rate.sleep()

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

def sail_ctrl():
    global curent_heading
    # receber posicaoo do vento (no ref do veleiro)
    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')
    global_dir = math.atan2(y,x)
    rospy.loginfo("valor de wind_dir = %f", math.degrees(global_dir))
    rospy.loginfo("valor de current_heading = %f", math.degrees(curent_heading))
    wind_dir = global_dir - curent_heading
    
    rospy.loginfo("valor de wind_dir = %f", math.degrees(wind_dir))
    #rospy.loginfo("valor de pi/2 = %f", math.pi/2)
    #rospy.loginfo("valor de wind_dir/pi/2 = %f", wind_dir/math.pi/2)
    #rospy.loginfo("valor de sail_max - sail_min = %f", sail_max - sail_min)
    #rospy.loginfo("valor de (sail_max - sail_min) * (wind_dir/(math.pi/2)) = %f", (sail_max - sail_min) * (wind_dir/(math.pi/2)))
    #rospy.loginfo("valor de sail_min = %f", sail_min)
    sail_angle = sail_min + (sail_max - sail_min) * (wind_dir/(math.pi/2))
    rospy.loginfo("valor de result = %f", sail_angle)
    return sail_angle

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
    global curent_heading

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

    curent_heading = math.radians(target_angle)
    
    err = sp_angle - target_angle
    err = P(err) + I(err)

    teste = 5
    if target_distance < 8 and target_distance > 5:
        actuator_vel = 20 
    elif target_distance < 5:
        actuator_vel = 0 
    else:
	actuator_vel = 100

    rudder_angle = 90 * (-err/180)

    #log_msg = "sp: {0}; erro: {1}; x_atual: {2}; y_atual: {3}; x_destino: {4}; y_destino: {5}; distancia_destino: {6}, rudder_angle: {7}; target_angle: {8}" .format(sp_angle, err, initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y, target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_distance, rudder_angle, target_angle)

    #rospy.loginfo(log_msg)

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
