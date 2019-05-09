#!/usr/bin/env python

import numpy, time, matplotlib.pyplot, matplotlib.animation
from PIL import Image
from scipy import misc
from scipy import ndimage
import yaml
import math
import scipy
import rospy
import sys

from wind_current.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

from PIL import Image
import matplotlib.pyplot as plt
from osgeo import gdal
import struct
from std_msgs.msg import Int64

fieldsVX = []
fieldsVY = []
fieldsVZ = []
fieldNames = []
pkg_path = "~/usv_sim_ws/src/usv_sim_lsa/usv_env_disturbs/wind_current/"

originX = 0
originY = 0
resolution = 1
obstacle_image = "no.jpg"
originX = 0
originY = 0
originZ = 0
cellSizeX = 0
cellSizeY = 0
cellSizeZ = 0
sizeX = 0
sizeY = 0
sizeZ = 0
time = 0
showVehicleOnMap = 0 # mark on map the received requests

pub = rospy.Publisher('windflow', OccupancyGrid, queue_size=1)
mymap = OccupancyGrid();
minHeight= 0
maxHeight= 0
minColisionHeight = 0
maxColisionHeight = 0
windSpeed = 0


def defineTime(data):
	global time
	time = data.data
	print "\n New time defined: ", time
	loadMapVX(time, 0)


def handleWindCurrent(req):
	global originX, originY, originZ, cellSizeX, cellSizeY, cellSizeZ
	global fieldsVX, fieldsVY, fieldsVZ, time, sizeX, sizeY, sizeZ
	#print "\n Received request",req.x,req.y,req.z
	x = (req.x-originX)/cellSizeX
	if (x < 0):
		x = 0
	if (x >= sizeX):
		x = sizeX-1
	y = (req.y-originY)/cellSizeY
	if (y < 0):
		y = 0
	if (y >= sizeY):
		y = sizeY-1
	z = (req.z-originZ)/cellSizeZ
	if (z < 0):
		z = 0
	if (z >= sizeZ):
		z = sizeZ-1
	
	#barrier[y][x] = True
	#barrierN = numpy.roll(barrier,  1, axis=0)				# sites just north of barriers
	#barrierS = numpy.roll(barrier, -1, axis=0)				# sites just south of barriers
	#barrierE = numpy.roll(barrier,  1, axis=1)				# etc.
	#barrierW = numpy.roll(barrier, -1, axis=1)
	#barrierNE = numpy.roll(barrierN,  1, axis=1)
	#barrierNW = numpy.roll(barrierN, -1, axis=1)
	#barrierSE = numpy.roll(barrierS,  1, axis=1)
	#barrierSW = numpy.roll(barrierS, -1, axis=1)

	x = int(x)
	y = int(y)
	z = int(z)
	#print "\n[",x,",",y,",",z,"]: (",fieldsVX[time][x][y][z],",",fieldsVY[time][x][y][z],",",fieldsVZ[time][x][y][z],")"
	if (showVehicleOnMap):
		#print "vehicle on ",(y*sizeY+x),"/",len(mymap.data), " pos ",x,y,z
		mymap.data[y*sizeX+x] = int(127)
	#	print "\n mymap: "
	#	print mymap.data
	return  GetSpeedResponse(fieldsVX[time][x][y][z], fieldsVY[time][x][y][z], fieldsVZ[time][x][y][z])
#        return GetSpeedResponse(ux[req.x][req.y], uy[req.x][req.y])


def parse_config_file(config_file_name):
	global pkg_path, fieldNames, fieldsVX, fieldsVY, fieldsVZ
	global originX, originY, originZ, cellSizeX, cellSizeY, cellSizeZ
	global sizeX, sizeY, sizeZ, time
	with open(pkg_path+"/"+config_file_name, 'r') as stream:
        	data_loaded = yaml.load(stream)
	print ('--------------------- Loading yaml file ', config_file_name)
	print ('--------------------- data_loaded: ', data_loaded)
	sizeX=data_loaded['size'][0]
	sizeY=data_loaded['size'][1]
	sizeZ=data_loaded['size'][2]
	originX=data_loaded['origin'][0]
	originY=data_loaded['origin'][1]
	originZ=data_loaded['origin'][2]
	cellSizeX=data_loaded['cellsize'][0]
	cellSizeY=data_loaded['cellsize'][1]
	cellSizeZ=data_loaded['cellsize'][2]
	time = data_loaded['time']
	if (time > len(data_loaded['fields'])):
		time = len(data_loaded['fields'])
	print "time: ",time
	print sizeX
	print sizeY
	print sizeZ

	for i in range(0, len(data_loaded['fields'])):
	   fieldNames.append(pkg_path+"/"+data_loaded['fields'][i])
	   fieldsVX.append([])
	   fieldsVY.append([])
	   fieldsVZ.append([])
	   fieldsVX[i]=numpy.resize(fieldsVX[i], (sizeX,sizeY,sizeZ));
	   fieldsVY[i]=numpy.resize(fieldsVY[i], (sizeX,sizeY,sizeZ));
	   fieldsVZ[i]=numpy.resize(fieldsVZ[i], (sizeX,sizeY,sizeZ));

	

	#fields[0] = data_loaded['fields'][0];
	#fields[1] = data_loaded['fields'][1];
	
def loadField(id):
    global fieldNames
    global fieldsVX, fieldsVY, fieldsVZ
    global originX, originY, originZ, cellSizeX, cellSizeY, cellSizeZ
    with open(fieldNames[id], 'r') as stream:
	first = True

        print "Processing lines of ",fieldNames[id]," . Wait..."
        for line in stream:
	    if(first): 
		first = False
		#print "Skipping header"
	    else:
                parts = line.split(",")
                vX = float(parts[1]);
                vY = float(parts[2]);
                vZ = float(parts[3]);
                x = int((float(parts[4])-originX)/cellSizeX);
                y = int((float(parts[5])-originY)/cellSizeY);
                z = int((float(parts[6])-originZ)/cellSizeZ);
#                print "DATA[",id,"]: ",vX," ",vY," ",vZ," ",x," ",y," ",z," "
#		[x][y][z]=[vX,vY,vZ]	
		fieldsVX[id][x][y][z]=vX
		fieldsVY[id][x][y][z]=vY
		fieldsVZ[id][x][y][z]=vZ
#		while (len(fields[id]) < x):
#			fields[id].extend(0)



	



def startRosService():
        s = rospy.Service('windCurrent', GetSpeed, handleWindCurrent)
        print "Ready to answer wind current. Wait full load before changing time"

def startTopicChangeTime():
	rospy.Subscriber("windCurrentTime", Int64, defineTime)
        print "Ready to change time."

def loadMapVX(id, z):
	global array, width, height, originX, originY, mymap, minSpeed, maxSpeed
	global sizeX, sizeY
	global fieldsVX, cellSizeX
	mynorm = matplotlib.colors.Normalize(vmin=-0.1,vmax=0.1);

	mymap.header.frame_id="world";
	mymap.info.resolution = cellSizeX;
	mymap.info.width = sizeX;
	mymap.info.height = sizeY;
	mymap.info.origin.position.x = originX;
	mymap.info.origin.position.y = originY;
	mymap.info.origin.position.z = 0;
	mymap.info.origin.orientation.x = 0;
	mymap.info.origin.orientation.y = 0;
	mymap.info.origin.orientation.z = 0;
	mymap.info.origin.orientation.w = 1;
	rospy.loginfo ("################### Preparing map. Size (%d, %d) origin(%d, %d)", sizeX, sizeY, originX, originY)
	print len(mymap.data), " == ", sizeX*sizeY
	if (len(mymap.data) == sizeX*sizeY):
		maior = -1000
		for y in xrange(sizeY-1, -1, -1):
			for x in range(0, sizeX):
				mymap.data[y*sizeX+x] = int(fieldsVX[id][x][y][z]);
				if (maior < mymap.data[y*sizeX+x]):
					maior =mymap.data[y*sizeX+x]
		for y in xrange(sizeY-1, -1, -1):
			for x in range(0, sizeX):
				mymap.data[y*sizeX+x] = 127*mymap.data[y*sizeX+x]/maior
	else:		
		for y in xrange(sizeY-1, -1, -1):
			for x in range(0, sizeX):
				mymap.data.append(int(fieldsVX[id][x][y][z]));

	rospy.loginfo("#################### Map loaded!")

if __name__ == '__main__':
	rospy.init_node('wind_current')
	rospy.loginfo ("starting wind_current. argument count: %d", len(sys.argv))
	if (len(sys.argv)!=3):
		rospy.logerr ("###################Error: argument count invalid! %d", len(sys.argv))
		for i in range(0, len(sys.argv)):
			rospy.logerr(" argv %d : %s", i, sys.argv[i])
		exit(0)

	pkg_path = sys.argv[1]
	config_file_name = sys.argv[2]
	parse_config_file(config_file_name)
	loadField(time)
	startRosService();
	loadMapVX(time, 0)
	for i in range(0, len(fieldNames)):
		if (i <> time):
			loadField(i)

	startTopicChangeTime()
	rate = rospy.Rate(30) # 10hz

	while not rospy.is_shutdown():
		try:
#		print ("waiting[",time,"] ")
#		rospy.logerr("---sending map! %d, %d", mymap.info.origin.position.x, mymap.info.origin.position.y)
			pub.publish(mymap);

			rate.sleep()
		except rospy.ROSInterruptException:
			rospy.logerr("ROS InterruptException! Just ignore the exception!") 
		except rospy.ROSTimeMovedBackwardsException:
			rospy.logerr("ROS Time Backwards! Just ignore the exception!")
	exit(0)
	#loadColisions()
	#initilizeArray()
	#startRosService();
	#loadMap()
	#rate = rospy.Rate(30) # 10hz
	#interval = 0
	#animate = matplotlib.animation.FuncAnimation(theFig, nextFrame, interval=1, blit=True)
	#matplotlib.pyplot.show()
	#while not rospy.is_shutdown():
	#	rospy.loginfo("Updating wind map");
	#	nextFrame(interval)
	#	interval+=1
#		rospy.logerr("---sending map! %d, %d", mymap.info.origin.position.x, mymap.info.origin.position.y)
#		pub.publish(mymap);
#		rospy.logerr("---map sent!")
	#	rate.sleep()
	#	print("end sleep")

	print "Closing wind_current."


	
