#!/usr/bin/env python

# Basic ROS imports
import roslib 
roslib.load_manifest('underwater_vehicle_dynamics')
import rospy
import PyKDL
import sys

# import msgs
from std_msgs.msg import Float64MultiArray 
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped

#import services
from std_srvs.srv import Empty

# More imports
from numpy import *
import tf

class Dynamics :

    
    def getConfig(self) :
        """ Load parameters from the rosparam server """
        self.num_actuators = rospy.get_param(self.vehicle_name+"/num_actuators")
        
        self.period = rospy.get_param(self.vehicle_name + "/dynamics" + "/period")
        self.mass = rospy.get_param(self.vehicle_name + "/dynamics" + "/mass")
        self.gravity_center = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/gravity_center"))
        self.g = rospy.get_param(self.vehicle_name + "/dynamics" + "/g")
        self.radius = rospy.get_param(self.vehicle_name + "/dynamics" + "/radius")
        self.ctf = rospy.get_param(self.vehicle_name + "/dynamics" + "/ctf")
        self.ctb = rospy.get_param(self.vehicle_name + "/dynamics" + "/ctb")
        self.actuators_tau = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_tau")
        self.actuators_maxsat= rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_maxsat")
        self.actuators_minsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_minsat")
        self.actuators_gain = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_gain")
        self.dzv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dzv")
        self.dv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dv")
        self.dh = rospy.get_param(self.vehicle_name + "/dynamics" + "/dh")
        self.density = rospy.get_param(self.vehicle_name + "/dynamics" + "/density")
        self.tensor = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/tensor"))
        self.damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/damping"))
        self.quadratic_damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/quadratic_damping"))
      
	self.am=rospy.get_param(self.vehicle_name + "/dynamics"+"/allocation_matrix")
  
        self.p_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_pose"))
        self.v_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_velocity"))
        self.frame_id = rospy.get_param(self.vehicle_name + "/dynamics" + "/frame_id")
        self.external_force_topic = rospy.get_param(self.vehicle_name + "/dynamics" + "/external_force_topic")
    
#       Currents data
        self.current_mean = array( rospy.get_param("dynamics/current_mean") )
        self.current_sigma = array( rospy.get_param("dynamics/current_sigma") )
        self.current_min = array( rospy.get_param("dynamics/current_min") )
        self.current_max = array( rospy.get_param("dynamics/current_max") )
        
        self.uwsim_period = rospy.get_param(self.vehicle_name + "/dynamics/uwsim_period")

        
    def s(self, x) :
        """ Given a 3D vector computes the 3x3 antisymetric matrix """
#        rospy.loginfo("s(): \n %s", x)
        ret = array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0 ])
        return ret.reshape(3,3)


    def generalizedForce(self, du):
	""" Computes the generalized force as B*u, being B the allocation matrix and u the control input """
        ct = zeros(len(du))
        i1 = nonzero(du >= 0.0)
        i2 = nonzero(du <= 0.0)
        ct[i1] = self.ctf
        ct[i2] = self.ctb
        
	#Evaluates allocation matrix loaded as parameter
	b=eval(self.am)
        b=array(b).reshape(6,size(b)/6)
      
        # t = generalized force
        t = dot(b, du)
        t = squeeze(asarray(t)) #Transforms a matrix into an array
        return t


    def coriolisMatrix(self):
        s1 = self.s(dot(self.M[0:3,0:3], self.v[0:3]) + dot(self.M[0:3,3:6], self.v[3:6]))
        s2 = self.s(dot(self.M[3:6,0:3], self.v[0:3]) + dot(self.M[3:6,3:6], self.v[3:6])) 
        c = zeros((6, 6))
        c[0:3,3:6] = -s1
        c[3:6,0:3] = -s1
        c[3:6,3:6] = -s2
        return c
    
    def dumpingMatrix(self):
        # lineal hydrodynamic damping coeficients  
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]
        
        # quadratic hydrodynamic damping coeficients
        Xuu = self.quadratic_damping[0]    #[Kg/m]
        Yvv = self.quadratic_damping[1]    #[Kg/m]
        Zww = self.quadratic_damping[2]    #[Kg/m]
        Kpp = self.quadratic_damping[3]    #[Kg*m*m]
        Mqq = self.quadratic_damping[4]    #[Kg*m*m]
        Nrr = self.quadratic_damping[5]    #[Kg*m*m]
    
        d = diag([Xu + Xuu*abs(self.v[0]), 
                  Yv + Yvv*abs(self.v[1]),
                  Zw + Zww*abs(self.v[2]),
                  Kp + Kpp*abs(self.v[3]),
                  Mq + Mqq*abs(self.v[4]),
                  Nr + Nrr*abs(self.v[5])])
        return d

    def gravity(self):
	""" Computes the gravity and buoyancy forces. Assumes a sphere model for now """
        #Weight and Flotability
        W = self.mass * self.g # [Kg]
        
        #If the vehicle moves out of the water the flotability decreases
	#FIXME: Assumes water surface at 0.0. Get this value from uwsim.
        if self.p[2] < 0.0: 
            r = self.radius + self.p[2]
            if r < 0.0:
                r = 0.0
        else :
            r = self.radius

	#TODO: either set as parameter, since different functions may be desired for different vehicles             
	#      or define common models and let the user choose one by the name
	#      Eventually let this part to bullet inside uwsim (HfFluid)
        F = ((4 * math.pi * pow(r,3))/3)*self.density*self.g 
  
        # gravity center position in the robot fixed frame (x',y',z') [m]
        zg = self.gravity_center[2]
        
        g = array([(W - F) * sin(self.p[4]),
                   -(W - F) * cos(self.p[4]) * sin(self.p[3]),
                   -(W - F) * cos(self.p[4]) * cos(self.p[3]),
                   zg*W*cos(self.p[4])*sin(self.p[3]),
                   zg*W*sin(self.p[4]),
                   0.0])
        
        return g
        
        
    def inverseDynamic(self) :
        """ Given the setpoint for each thruster, the previous velocity and the 
            previous position computes the v_dot """
        du = self.thrustersDynamics(self.u)
        t = self.generalizedForce(du)
        c = self.coriolisMatrix()
        d = self.dumpingMatrix()
        g = self.gravity()
        c_v = dot((c-d), self.v)
        v_dot = dot(self.IM, (t-c_v-g+self.collisionForce)) #t-c_v-g+collisionForce
        v_dot = squeeze(asarray(v_dot)) #Transforms a matrix into an array
        self.collisionForce=[0,0,0,0,0,0]
        return v_dot
        
#
    
    def integral(self, x_dot, x, t) :
        """ Computes the integral o x dt """
        return (x_dot * t) + x
    
    
    def kinematics(self) :
        """ Given the current velocity and the previous position computes the p_dot """
        roll = self.p[3]
        pitch = self.p[4]
        yaw = self.p[5]
        
        rec = [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch),
               sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -cos(yaw)*sin(roll)+sin(pitch)*sin(yaw)*cos(roll),
               -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
        rec = array(rec).reshape(3,3)
        
        to = [1.0, sin(roll)*tan(pitch), cos(roll)*tan(pitch),
              0.0, cos(roll), -sin(roll),
              0.0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]
        to = array(to).reshape(3,3)
        
        p_dot = zeros(6)
        p_dot[0:3] = dot(rec, self.v[0:3])
        p_dot[3:6] = dot(to, self.v[3:6])
        return p_dot

    def updateThrusters(self, thrusters) :
    	"""Receives the control input, saturates each component to maxsat or minsat, and multiplies each component by the actuator gain"""
	#TODO: Check the size of thrusters.data
        t = array(thrusters.data)
        for i in range(size(t)):
	   if t[i]>self.actuators_maxsat[i]:
		t[i]=self.actuators_maxsat[i]
	   elif t[i]<self.actuators_minsat[i]:
		t[i]=self.actuators_minsat[i]
	self.u=t
        for i in range(size(t)):
           self.u[i] = self.u[i]*self.actuators_gain[i]
        
        
    def thrustersDynamics(self, u):
        y = zeros(size(u))
        for i in range(size(u)):
            y[i] = (self.period * u[i] + self.actuators_tau[i] * self.y_1[i]) / (self.period + self.actuators_tau[i])
            
        self.y_1 = y
        return y
  
    def updateCollision(self, force) :
        self.collisionForce=[force.wrench.force.x,force.wrench.force.y,force.wrench.force.z,force.wrench.torque.x,force.wrench.torque.y,force.wrench.torque.z]        
    
    def pubPose(self, event):
        pose = Pose()
        
        pose.position.x = self.p[0]
        pose.position.y = self.p[1]
        pose.position.z = self.p[2]
        
        orientation = tf.transformations.quaternion_from_euler(self.p[3], self.p[4], self.p[5], 'sxyz')
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
            
        self.pub_pose.publish(pose)
     
        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.p[0], self.p[1], self.p[2]), orientation, 
        rospy.Time.now(), "world", str(self.frame_id))
    
    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame

    def reset(self,req):
        self.v = self.v_0
        self.p = self.p_0
	return []
    
    def __init__(self):
        """ Simulates the dynamics of an AUV """

        if len(sys.argv) != 6: 
          sys.exit("Usage: "+sys.argv[0]+" <namespace> <input_topic> <output_topic>")
 
        self.namespace=sys.argv[1]
        self.vehicle_name=self.namespace
        self.input_topic=sys.argv[2]
        self.output_topic=sys.argv[3]

    #  Collision parameters
	self.collisionForce = [0,0,0,0,0,0]

    #   Load dynamic parameters
        self.getConfig()
        #self.altitude = -1.0 
        self.y_1 = zeros(5)
        
    #   Create publisher
        self.pub_pose= rospy.Publisher(self.output_topic, Pose)
        rospy.init_node("dynamics_"+self.vehicle_name)
                
    #   Init pose and velocity and period
        self.v = self.v_0
        self.p = self.p_0
        
        # Inertia Tensor. Principal moments of inertia, and products of inertia [kg*m*m]
        Ixx = self.tensor[0]
        Ixy = self.tensor[1] 
        Ixz = self.tensor[2]
        Iyx = self.tensor[3]
        Iyy = self.tensor[4]
        Iyz = self.tensor[5]
        Izx = self.tensor[6]
        Izy = self.tensor[7]
        Izz = self.tensor[8] 
        m = self.mass
        xg = self.gravity_center[0]
        yg = self.gravity_center[1]
        zg = self.gravity_center[2]
        
        Mrb = rospy.get_param(self.vehicle_name + "/dynamics" + "/Mrb")
        Mrb = array(Mrb).reshape(6, 6)
             
        # Inertia matrix of the rigid body
        # Added Mass derivative
        Ma = rospy.get_param(self.vehicle_name + "/dynamics" + "/Ma")
        Ma = array(Ma).reshape(6, 6) 
        
        self.M = Mrb + Ma    # mass matrix: Mrb + Ma
        self.IM = matrix(self.M).I
#        rospy.loginfo("Inverse Mass Matrix: \n%s", str(self.IM))
              
        #Init currents
        random.seed()
        self.e_vc = self.current_mean 
	#The number of zeros will depend on the number of actuators
        self.u = array(zeros(self.num_actuators)) # Initial thrusters setpoint

    	#Publish pose to UWSim
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubPose)
        
    #   Create Subscribers for thrusters and collisions
	#TODO: set the topic names as parameters
        rospy.Subscriber(self.input_topic, Float64MultiArray, self.updateThrusters)
        rospy.Subscriber(self.external_force_topic, WrenchStamped, self.updateCollision)


	s = rospy.Service('/dynamics/reset',Empty, self.reset)
	
    def iterate(self):
        t1 = rospy.Time.now()

        # Main loop operations
        self.v_dot = self.inverseDynamic()
        self.v = self.integral(self.v_dot, self.v, self.period)
        self.p_dot = self.kinematics()
        self.p = self.integral(self.p_dot, self.p, self.period)

        t2 = rospy.Time.now()
        p = self.period - (t2-t1).to_sec()
        if p < 0.0 : p = 0.0
        rospy.sleep(p)
        

if __name__ == '__main__':
    try:
        dynamics = Dynamics() 
        while not rospy.is_shutdown():
            dynamics.iterate()

    except rospy.ROSInterruptException: pass
    
