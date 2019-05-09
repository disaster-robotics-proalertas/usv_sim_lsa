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
Kp = 1 
Ki = 0 
result = Float64()
result.data = 0
windDir= Float64()
windDir.data = 1.5 
currentHeading= Float64()
currentHeading.data = 0
f_distance = 4 
sail_min = 0 
sail_max = math.pi/2
current_heading = 0
rudder_min = -math.pi/3
rudder_max = math.pi/3
rudder_med = (rudder_min + rudder_max)/2
heeling = 0
spHeading = 10 
isTacking = 0

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
    global f_distance
    if target_distance < f_distance:
        result.data = 1
    if target_distance >= f_distance:    
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
    global currentHeading
    global windDir 
    global isTacking
    global heeling
    global spHeading

    rospy.init_node('usv_simple_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value) # 10h
    # publishes to thruster and rudder topics
    #pub_sail = rospy.Publisher('angleLimits', Float64, queue_size=10)
    pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)
    pub_result = rospy.Publisher('move_usv/result', Float64, queue_size=10)
    pub_heading = rospy.Publisher('currentHeading', Float64, queue_size=10)
    pub_windDir = rospy.Publisher('windDirection', Float64, queue_size=10)
    pub_heeling = rospy.Publisher('heeling', Float64, queue_size=10)
    pub_spHeading = rospy.Publisher('spHeading', Float64, queue_size=10)
    
    # subscribe to state and targer point topics
    rospy.Subscriber("state", Odometry, get_pose)  # get usv position (add 'gps' position latter)
    rospy.Subscriber("move_usv/goal", Odometry, get_target)  # get target position

    while not rospy.is_shutdown():
        try:
            pub_rudder.publish(rudder_ctrl_msg())
            #pub_sail.publish(sail_ctrl())
            pub_result.publish(verify_result())
            pub_heading.publish(currentHeading)
            pub_windDir.publish(windDir)
            pub_heeling.publish(heeling)
            pub_spHeading.publish(spHeading)
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

def sail_ctrl():
    global current_heading
    global windDir
    global heeling
    # receber posicaoo do vento (no ref do veleiro)
    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')
    global_dir = math.atan2(y,x)
    heeling = angle_saturation(math.degrees(global_dir)+180)
    #rospy.loginfo("valor de wind_dir = %f", math.degrees(windDir))
    #rospy.loginfo("global_dir = %f", math.degrees(global_dir))
    #rospy.loginfo("current_heading = %f", math.degrees(current_heading))
    wind_dir = global_dir - current_heading
    wind_dir = angle_saturation(math.degrees(wind_dir)+180)
    windDir.data = math.radians(wind_dir)

    #rospy.loginfo("wind_dir = %f", wind_dir)
    #rospy.loginfo("valor de pi/2 = %f", math.pi/2)
    #rospy.loginfo("valor de wind_dir/pi/2 = %f", wind_dir/math.pi/2)
    #rospy.loginfo("valor de sail_max - sail_min = %f", sail_max - sail_min)
    #rospy.loginfo("valor de (sail_max - sail_min) * (wind_dir/(math.pi/2)) = %f", (sail_max - sail_min) * (wind_dir/(math.pi/2)))
    #rospy.loginfo("valor de sail_min = %f", sail_min)
    #sail_angle = sail_min + (sail_max - sail_min) * (wind_dir/180)
    
    sail_angle = math.radians(wind_dir)/2;
    if math.degrees(sail_angle) < -80:
        sail_angle = -sail_angle
    #if sail_angle < 0:
    #    sail_angle = -sail_angle
#    rospy.loginfo("sail angle = %f", math.degrees(sail_angle))
    return -sail_angle

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
    global current_heading
    global currentHeading
    global spHeading

    x1 = initial_pose.pose.pose.position.x
    y1 = initial_pose.pose.pose.position.y
    x2 = target_pose.pose.pose.position.x
    y2 = target_pose.pose.pose.position.y

    # encontra angulo ate o ponto de destino (sp)
    myradians = math.atan2(y2-y1,x2-x1)
    sp_angle = math.degrees(myradians)

    target_distance = math.hypot(x2-x1, y2-y1) 
    #rospy.loginfo("target_distance: %f", target_distance)

    # encontra angulo atual
    # initial_pose.pose.pose.orientation = nav_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))
    
    quaternion = (initial_pose.pose.pose.orientation.x, initial_pose.pose.pose.orientation.y, initial_pose.pose.pose.orientation.z,initial_pose.pose.pose.orientation.w) 
    euler = tf.transformations.euler_from_quaternion(quaternion) 

    # target_angle = initial_pose.pose.pose.orientation.yaw
    target_angle = math.degrees(euler[2])
    #rospy.loginfo("target_angle %f", target_angle)

#    rospy.loginfo(sp_angle)
    sp_angle = angle_saturation(sp_angle)
    spHeading = sp_angle
    sp_angle = -sp_angle
#    rospy.loginfo("sp: %f", sp_angle)
    target_angle = angle_saturation(target_angle)
    target_angle = -target_angle
#    rospy.loginfo("target_angle: %f", target_angle)

    current_heading = math.radians(target_angle)
    currentHeading.data = current_heading
    
    err = sp_angle - target_angle
    err = angle_saturation(err)
    err = P(err) + I(err)

    rudder_angle = err/2

    if err > 60:
        err = 60
    if err < -60:
        err = -60

#    if err > 180:
#        err = 180
#    if err < -180:
#        err = -180
#
#    if err < 0:
#        rudder_angle = rudder_med + (rudder_min - rudder_med) * ((err) / 180)
#    else:
#        rudder_angle = rudder_med + (rudder_max - rudder_med) * (-err / 180)

    log_msg = "sp: {0}; erro: {1}; x_atual: {2}; y_atual: {3}; x_destino: {4}; y_destino: {5}; distancia_destino: {6}, rudder_angle: {7}; target_angle: {8}" .format(sp_angle, err, initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y, target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_distance, rudder_angle, target_angle)

    #rospy.loginfo(log_msg)

    return math.radians(rudder_angle)

def rudder_ctrl_msg():
    msg = JointState()
    msg.header = Header()
    msg.name = ['rudder_joint', 'sail_joint']
    msg.position = [rudder_ctrl(), sail_ctrl()]
    msg.velocity = []
    msg.effort = []
    return msg

if __name__ == '__main__':
    try:
        talker_ctrl()
    except rospy.ROSInterruptException:
        pass
