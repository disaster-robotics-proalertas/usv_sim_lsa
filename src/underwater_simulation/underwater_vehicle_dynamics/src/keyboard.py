#!/usr/bin/env python

from std_msgs.msg import Float64MultiArray 
import termios, fcntl, sys, os
import rospy

#import services
from std_srvs.srv import Empty

if len(sys.argv) != 4: 
	sys.exit("Usage: "+sys.argv[0]+" <thrusters_topic>")
 

thrusters_topic=sys.argv[1]

fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

pub = rospy.Publisher(thrusters_topic, Float64MultiArray)
rospy.init_node('keyboard')
rospy.wait_for_service('/dynamics/reset')
reset=rospy.ServiceProxy('/dynamics/reset', Empty)
try:
    while not rospy.is_shutdown():
        thrusters=[0,0,0,0,0]
	msg = Float64MultiArray()
        try:
            c = sys.stdin.read(1)
            ##print "Got character", repr(c)
	    if c=='w':
		thrusters[0]=thrusters[1]=0.4
	    elif c=='s':
		thrusters[0]=thrusters[1]=-0.4
	    elif c=='a':
		thrusters[4]=0.4
	    elif c=='d':
		thrusters[4]=-0.4
	    elif c==' ':
		reset()
	    elif c=='\x1b':
		c2= sys.stdin.read(1)
		c2= sys.stdin.read(1)
	        if c2=='A':
		    thrusters[2]=thrusters[3]=0.4
	        elif c2=='B':
		    thrusters[2]=thrusters[3]=-0.4
	        elif c2=='C':
		    thrusters[0]=-0.4
		    thrusters[1]=0.4
	        elif c2=='D':
		    thrusters[0]=0.4
		    thrusters[1]=-0.4
	    else:
		print 'wrong key pressed'
	    while c!='':
	        c = sys.stdin.read(1)
        except IOError: pass
	msg.data = thrusters
        pub.publish(msg)
	rospy.sleep(0.1)
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
