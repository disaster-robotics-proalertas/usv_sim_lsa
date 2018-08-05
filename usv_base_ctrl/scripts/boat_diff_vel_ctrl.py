#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import tf
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion

class VelocityCtrl():
    def thruster_ctrl_msg(self, cmd, name):
        self.thruster_msg.name = ['fwd_left', 'fwd_right']
        self.thruster_msg.position = [self.vel_left, self.vel_right]
        self.thruster_msg.velocity = []
        self.thruster_msg.effort = []
        # rospy.loginfo("Velocidade: {0}", self.thruster_msg)
        return self.thruster_msg

    def get_usv_vel(self, usv_vel_tmp):
        self.usv_vel = usv_vel_tmp 

    def get_target_vel(self, target_vel_tmp):
        self.target_vel_ant = self.target_vel
        self.target_vel = target_vel_tmp

    def __init__(self):
        rospy.init_node('usv_vel_ctrl', anonymous=False)
        self.rate = 10
        r = rospy.Rate(self.rate) # 10hertz

        self.usv_vel = Odometry()
        self.usv_vel_ant = Odometry()
        self.target_vel = Twist()
        self.target_vel_ant = Twist()
        self.thruster_msg = JointState()
        self.thruster_msg.header = Header()
        self.kp_lin = 80
        self.ki_lin = 200
        thruster_name = 'fwd_left, fwd_right'
        self.I_ant_lin = 0
        self.I_ant_ang = 0
        self.lin_vel = 0
        self.lin_vel_ang = 0
        self.ang_vel = 0
        self.kp_ang = 80 
        self.ki_ang = 100
        self.thruster_max = 30
        self.vel_left = 0
        self.vel_right = 0
        self.thruster_command = numpy.array([0, 0])
        self.erro = 0

        self.pub_motor = rospy.Publisher('thruster_command', JointState, queue_size=10)

        rospy.Subscriber("state", Odometry, self.get_usv_vel)  
        rospy.Subscriber("cmd_vel", Twist, self.get_target_vel)

        while not rospy.is_shutdown():
            self.pub_motor.publish(self.thruster_ctrl_msg(self.vel_ctrl(), thruster_name))
            r.sleep()

    def vel_ctrl(self):
        self.lin_vel = self.target_vel.linear.x - self.usv_vel.twist.twist.linear.x
        self.lin_vel = self.lin_vel * self.kp_lin + self.I_lin(self.lin_vel)

        #if self.target_vel.angular.z != self.target_vel_ant.angular.z:
        #    self.I_ant_ang = 0

        if self.target_vel.angular.z == 0:
            self.vel_left = self.lin_vel
            self.vel_right = self.lin_vel
        else: 
            self.ang_vel = self.target_vel.angular.z - self.usv_vel.twist.twist.angular.z
            self.erro = self.ang_vel
            self.ang_vel = self.ang_vel * self.kp_ang + self.I_ang(self.ang_vel)
            self.vel_left = self.lin_vel - self.ang_vel
            self.vel_right = self.lin_vel + self.ang_vel
                   
        self.vel_left = self.sat_thruster(self.vel_left)
        self.vel_right = self.sat_thruster(self.vel_right)
        msg = "atual: {0}; desejada: {1}; erro: {2}; vel_left: {3}; vel_right: {4}; erro_bruto: {5}" .format(self.usv_vel.twist.twist.angular.z, self.target_vel.angular.z, self.ang_vel, self.vel_left, self.vel_right, self.erro)
        rospy.loginfo(msg)

        return [self.vel_left, self.vel_right]

    def I_lin(self, erro):
        self.I_ant_lin = self.I_ant_lin + erro * self.ki_lin * 1/self.rate
        return self.I_ant_lin

    def sat_thruster(self, thruster):
        if thruster > self.thruster_max:
            thruster = self.thruster_max
        if thruster < -self.thruster_max:
            thruster = -self.thruster_max
        return thruster

    def I_ang(self, erro):
        self.I_ant_ang = self.I_ant_ang + erro * self.ki_ang * 1/self.rate
        self.I_ant_ang = self.sat_thruster(self.I_ant_ang)
        return self.I_ant_ang
    
if __name__ == '__main__':
    try:
        VelocityCtrl()
    except rospy.ROSInterruptException:
        rospy.loginfo("VelocityCtrl node terminated.")
