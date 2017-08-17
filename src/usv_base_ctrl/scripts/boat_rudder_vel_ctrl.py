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

    def rudder_ctrl_msg(self, cmd, name):
        self.rudder_msg.name = [name]
        self.rudder_msg.position = [cmd]
        self.rudder_msg.velocity = []
        self.rudder_msg.effort = []
        # rospy.loginfo("Velocidade: {0}", self.thruster_msg)
        return self.rudder_msg

    def get_usv_vel(self, usv_vel_tmp):
        if usv_vel_tmp != self.usv_vel_ant:
            self.usv_vel_ant = self.usv_vel
            self.usv_vel = usv_vel_tmp 


    def get_target_vel(self, target_vel_tmp):
        if target_vel_tmp != self.target_vel_ant:
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
        self.rudder_msg = JointState()
        self.thruster_msg.header = Header()
        self.rudder_msg.header = Header()
        self.kp_lin = 80
        self.ki_lin = 200
        thruster_name = 'fwd'
        rudder_name = 'rudder_joint'
        self.I_ant_lin = 0
        self.I_ant_ang = 0
        self.lin_vel = 0
        self.lin_vel_ang = 0
        self.ang_vel = 0
        self.kp_ang = 2 
        self.ki_ang = 4
        self.rudder_max = 70
        self.thruster_max = 30

        self.pub_motor = rospy.Publisher('thruster_command', JointState, queue_size=10)
        self.pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)

        rospy.Subscriber("state", Odometry, self.get_usv_vel)  
        rospy.Subscriber("cmd_vel", Twist, self.get_target_vel)

        while not rospy.is_shutdown():
            self.pub_rudder.publish(self.rudder_ctrl_msg(self.ang_vel_ctrl(), rudder_name))
            self.pub_motor.publish(self.thruster_ctrl_msg(self.lin_vel_ctrl(), thruster_name))
            r.sleep()

    def lin_vel_ctrl(self):
        if self.target_vel.linear.x > self.target_vel.angular.z*2:
            self.lin_vel = self.target_vel.linear.x - self.usv_vel.twist.twist.linear.x
        else:
            self.lin_vel = self.target_vel.angular.z*2 - self.usv_vel.twist.twist.linear.x

        self.lin_vel = self.lin_vel * self.kp_lin + self.I_lin(self.lin_vel)
        #msg = "atual: {0}; desejada: {1}; erro: {2}; I: {3};" .format(self.usv_vel.twist.twist.linear.x, self.target_vel.linear.x, self.lin_vel, self.I_ant_lin)
        #rospy.loginfo(msg)
        self.sat_thruster()
        return self.lin_vel

    def ang_vel_ctrl(self):
        self.ang_vel = self.target_vel.angular.z - self.usv_vel.twist.twist.angular.z
        self.ang_vel = self.ang_vel * self.kp_ang + self.I_ang(self.ang_vel)
        self.sat_rudder()
        #msg = "atual: {0}; desejada: {1}; erro: {2}; I: {3}; Comand motor: {4}; erro cominado: {5}; i_lin: {6}" .format(self.usv_vel.twist.twist.angular.z, self.target_vel.angular.z, self.ang_vel, self.I_ant_ang, self.lin_vel, self.lin_vel + self.I_ant_ang, self.I_ant_lin)
        #rospy.loginfo(msg)
        return -self.ang_vel

    def I_lin(self, erro):
        self.I_ant_lin = self.I_ant_lin + erro * self.ki_lin * 1/self.rate
        return self.I_ant_lin

    def I_ang(self, erro):
        self.I_ant_ang = self.I_ant_ang + erro * self.ki_ang * 1/self.rate
        return self.I_ant_ang

    def sat_thruster(self):
        if self.lin_vel > self.thruster_max:
            self.lin_vel = self.thruster_max
        if self.lin_vel < -self.thruster_max:
            self.lin_vel = -self.thruster_max

    def sat_rudder(self):
        if self.ang_vel > self.rudder_max:
            self.ang_vel = self.rudder_max
        if self.ang_vel < -self.rudder_max:
            self.ang_vel = -self.rudder_max
    
    def sat_thruster_I(self):
        if self.I_ant_lin > 2*self.thruster_max:
            self.I_ant_lin = 2*self.thruster_max
        if self.I_ant_lin < -2*self.thruster_max:
            self.I_ant_lin = -2*self.thruster_max

if __name__ == '__main__':
    try:
        VelocityCtrl()
    except rospy.ROSInterruptException:
        rospy.loginfo("VelocityCtrl node terminated.")
