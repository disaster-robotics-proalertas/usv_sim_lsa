import sys
import tf
import rosbag
from sklearn.metrics import mean_squared_error
from math import sqrt
import numpy
import time as tempo
import os
import locale
import matplotlib.pyplot as plt

locale.setlocale(locale.LC_ALL, "")

oldNtime =0
oldStime =0
y_actual = []
y_predicted = []
time = []
posY = []

posRefX = []
posRefY = []

lineType =[]
lineType.append('red')
lineType.append('blue')
lineType.append('green')
lineType.append('pink')
lineType.append('black')


if len(sys.argv) == 2:
	name = sys.argv[1]
if len(sys.argv) == 3:
	name = sys.argv[1]
	name2 = sys.argv[2]


for id in range(0, len(sys.argv)-1):
	name = sys.argv[id+1]
	print "\n processing ", name
	posX = []
	posY = []
	for topic, msg, t in rosbag.Bag(name).read_messages():
		posX.append(msg.pose.pose.position.x)
		posY.append(msg.pose.pose.position.y)
	plt.plot(posX, posY, color=lineType[id], label=name)

plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
axes = plt.gca()
#axes.axis('equal')
axes.axis('scaled')
#axes.set_xlim([xmin,xmax])
#axes.set_ylim([80,125])

plt.show()
