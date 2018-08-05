#!/usr/bin/env python


import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from nav_msgs.msg import Odometry
import math


fig, ax = plt.subplots(1,1,figsize=(5,3))
fig = plt.gcf()
fig.canvas.set_window_title('Speed (m/s) vs Time (s)')
#line, = ax.plot(x, np.sin(x))
#x = np.arange(0, 2*np.pi, 0.01)
x = []
y = []



def animate(i):
    global x, y
#    print i
#    x.append(i)
#    y.append(i*2)
    line, = ax.plot(x, y, color='b')
    #ax.axis([0,300,0,520])
    ax.axis([0,450,0,5])
#    line.set_ydata(x+i/10.0)  # update the data
    return line,

#def init():
#    line.set_ydata(np.ma.array(x, mask=True))
#    return line,


def callback(data):
	global x, y
	now = data.header.stamp.secs+0.1*(int(data.header.stamp.nsecs/100000000.0))
	if (len(x) == 0 or x[len(x)-1]<now):
		x.append(now)
#		v = math.sqrt(data.pose.pose.position.x*data.pose.pose.position.x+data.pose.pose.position.y*data.pose.pose.position.y)
		v = math.sqrt(data.twist.twist.linear.x*data.twist.twist.linear.x+data.twist.twist.linear.y*data.twist.twist.linear.y)
		y.append(v)
#	rospy.loginfo("I heard %s",data.pose.pose.position)
#	rospy.loginfo("I heard %s",data.twist.twist.linear)


if __name__ == '__main__':
	rospy.init_node('my_plot_graph')
	rospy.Subscriber("/rudderboat/state", Odometry, callback)
	interval = 0

	ani = animation.FuncAnimation(fig, animate, interval=1, blit=False)
	plt.show()

