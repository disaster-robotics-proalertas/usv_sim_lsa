#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion

class VelocityCtrl():
    def thruster_ctrl_msg(self, cmd, name):
        self.thruster_msg.name = [name]
        self.thruster_msg.position = [cmd]
        self.thruster_msg.velocity = []
        self.thruster_msg.effort = []
        # rospy.loginfo("Velocidade: {0}", self.thruster_msg)
        return self.thruster_msg

    def get_usv_vel(self, usv_vel_tmp):
        self.usv_vel = usv_vel_tmp 

    def get_target_vel(self, target_vel_tmp):
        self.target_vel = target_vel_tmp

    def __init__(self):
        rospy.init_node('usv_vel_ctrl', anonymous=False)
        self.rate = 10
        r = rospy.Rate(self.rate) # 10hertz

        self.usv_vel = Odometry()
        self.target_vel = Twist()
        self.thruster_msg = JointState()
        self.thruster_msg.header = Header()
        self.kp = 80
        thruster_name = 'fwd_left'
        self.ki = 200
        self.I_ant = 0
        self.lin_vel = 0
        self.ang_vel = 0

        self.pub_motor = rospy.Publisher('/barco_auv/thruster_command', JointState, queue_size=10)

        rospy.Subscriber("/barco_auv/state", Odometry, self.get_usv_vel)  
        rospy.Subscriber("/barco_auv/cmd_vel", Twist, self.get_target_vel)

        while not rospy.is_shutdown():
            self.pub_motor.publish(self.thruster_ctrl_msg(lin_vel_ctrl(), thruster_name))
            r.sleep()

    def lin_vel_ctrl(self):
        self.lin_vel = self.target_vel.linear.x - self.usv_vel.twist.twist.linear.x
        self.lin_vel = self.lin_vel * self.kp + self.I(self.lin_vel)
        msg = "atual: {0}; desejada: {1}; erro: {2}; I: {3};" .format(self.usv_vel.twist.twist.linear.x, self.target_vel.linear.x, self.lin_vel, self.I_ant)
        rospy.loginfo(msg)
        return self.lin_vel

    def ang_vel_ctrl(self):



    def I(self, erro):
        self.I_ant = self.I_ant + erro * self.ki * 1/self.rate
        return self.I_ant

    def quat_to_angle(quarternion):
        return math.degrees(tf.transformations.euler_from_quaternion(quaternion))


if __name__ == '__main__':
    try:
        VelocityCtrl()
    except rospy.ROSInterruptException:
        rospy.loginfo("VelocityCtrl node terminated.")
