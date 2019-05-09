#!/usr/bin/env python
import os
import sys
import wx,h5py
import numpy
import yaml
import thread
import time as threadTime
import rospy
from water_current.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist, Point, Quaternion
import math

filename='~/usv_sim_ws/src/usv_sim_lsa/usv_env_disturb/water_current/maps/t3.p02.hdf'
refX = 477092.4903
refY = 6675824.0
nodeCoordinate = 'FacePoints Coordinate'
nodeNameX = 'Node X Vel'
nodeNameY = 'Node Y Vel'
width = 1341
height = 558
mapTorviz = 1
startTime = 0
endTime = 0
repeat = -1
interval = 1

datasetCoord = []
datasetX = []
datasetY = []
time = 0
arrayX = []
arrayY = []
indexMap = []
showVehicleOnMap = 1;



pub = rospy.Publisher('waterflow', OccupancyGrid, queue_size=1)
mymap = OccupancyGrid();

def PrintHierarchy(gid, level, text):
	global nodeNameX, nodeNameY, datasetX, datasetY, nodeCoordinate, datasetCoord
	for hidStr in h5py.h5g.GroupIter(gid):
		lid=gid.links.get_info(hidStr)
		try:
			hid = h5py.h5o.open(gid,hidStr)
		except KeyError as e:
			hid=None
			t=None
		t=type(hid)
		
		if t==h5py.h5g.GroupID:
			PrintHierarchy(hid, level+1, text+" > "+hidStr)
		else:
			if hidStr == nodeNameX:
				if t==h5py.h5d.DatasetID:
					datasetX=h5py.Dataset(hid)
				elif t==np.ndarray:
				    	datasetX=hid
				text = text + "["+str(len(datasetX))+"]"

			if hidStr == nodeNameY:
				if t==h5py.h5d.DatasetID:
					datasetY=h5py.Dataset(hid)
				elif t==np.ndarray:
				    	datasetY=hid
				text = text + "["+str(len(datasetY))+"]"

			if hidStr == nodeCoordinate:
				if t==h5py.h5d.DatasetID:
					datasetCoord=h5py.Dataset(hid)
				elif t==np.ndarray:
				    	datasetCoord=hid
				text = text + "["+str(len(datasetCoord))+"]"
			print text+" > "+hidStr+" ["+str(len(hidStr))+"]"


def preprocessDataset2():
	global arrayX, arrayY, width, height, datasetCoord, datasetX, datasetY, refX, refY, indexMap

	arrayX = numpy.zeros((width,height),dtype=numpy.float64)
	arrayY = numpy.zeros((width,height),dtype=numpy.float64)
	indexMap = numpy.zeros((width,height),dtype=numpy.int)
	estimatedRefX = datasetCoord[0][0]
	estimatedRefY = datasetCoord[0][1]
	dataSetLen = len(datasetCoord)
	print "Preprocessing the dataset indexes(length: ",dataSetLen,"). This can take some minutes..."
	amount = len(datasetCoord)/100
	for i in range(0, dataSetLen):
		perc = i / amount
		sys.stdout.write('\r%d %% processed' %perc)

		if (estimatedRefX>datasetCoord[i][0]):
			estimatedRefX = datasetCoord[i][0];
		if (estimatedRefY>datasetCoord[i][1]):
			estimatedRefY = datasetCoord[i][1];
		x = (int)(math.floor(datasetCoord[i][0]-refX))
		y = (int)(math.floor(datasetCoord[i][1]-refY))
		if (x < 0):
			x = 0
		if (x >=width):
			x = width
		if (y < 0):
			y = 0
		if (y >= height):
			y = height
#		arrayX[x][y] = datasetX[time][i]
#		arrayY[x][y] = datasetY[time][i]
		indexMap[x][y] = i		
		#print "V[",x,y,"](",arrayX[x][y],arrayY[x][y],")"
	print "\nestimatedRefX: ", estimatedRefX
	print "estimatedRefY: ", estimatedRefY
	
def handleWaterCurrent2(req):
	global refX, refY, datasetX, datasetY, datasetCoord, time, indexMap, mymap, width, showVehicleOnMap
	i = indexMap[req.x][req.y]	
	print ("Received request",req.x, req.y," --> ",datasetX[time][i], datasetY[time][i])

	#for x in range (0, req.x):
	#   mymap.data[req.y*width+x]=120
	if (showVehicleOnMap):
		mymap.data[req.y*width+req.x]=127
	return GetSpeedResponse(datasetX[time][i], datasetY[time][i])

def defineTime(data):
	global time
	time = data.data
	loadMap()
	print "\n New time defined: ", time

def startRosService():
	rospy.Subscriber("waterCurrentTime", Int64, defineTime)
	preprocessDataset2()
        s = rospy.Service('waterCurrent', GetSpeed, handleWaterCurrent2)
        print "\nReady to answer water current.\n"



def loadMap():
	global arrayX, arrayY, width, height, originX, originY, mymap, filename, time

	mymap.header.frame_id="odom";
	mymap.info.resolution = resolution;
	mymap.info.width = width;
	mymap.info.height = height;
	mymap.info.origin.position.x = originX;
	mymap.info.origin.position.y = originY;
	mymap.info.origin.position.z = 0;
	mymap.info.origin.orientation.x = 0;
	mymap.info.origin.orientation.y = 0;
	mymap.info.origin.orientation.z = 0;
	mymap.info.origin.orientation.w = 1;
	rospy.logerr ("-################### Preparing map to rviz. Size (%d, %d) origin(%d, %d)", width, height, 0, 0)
#	for y in xrange(height-1, -1, -1):
#	mymap.data = []
   	mymap.data = [0] * width*height
	delta = 1.2
	for y in range(0, height):
		#rospy.logerr ('\r%d rows ' %y)
		#sys.stdout.write('\r%d rows ' %y)	
		#sys.stdout.flush()
		sys.stdout.write('\r%d rows processed' %y)
		sys.stdout.flush()
		for x in range(0, width):
			i = indexMap[x][y]
			#px = (int)(math.floor(datasetCoord[i][0]-refX))
			#py = (int)(math.floor(datasetCoord[i][1]-refY))
			value = 127*datasetX[time][i]/delta;
			if (value < 0):
				value = (-1)*value;
			if (value >127):
				value = 127;
			#mymap.data.append(int(value))
			mymap.data[y*width+x]=int(value);	
			#rospy.logerr ( value )
#	numpy.savetxt('data/'+str(filename)+'_'+str(time)+'.out', mymap.data, delimiter=',')		
	rospy.logerr("#################### Map loaded!")


def parse_config_file(config_file_name):
	global filename, refX, refY, nodeCoordinate, nodeNameX, nodeNameY, time, width, height, resolution, originX, originY, mapTorviz
	global startTime, endTime, repeat, interval

	with open(config_file_name, 'r') as stream:
        	data_loaded = yaml.load(stream)
	print ('--------------------- Loading yaml file ', config_file_name)
	originX = data_loaded['origin'][0];
	originY = data_loaded['origin'][1];
	refX = data_loaded['ref'][0];
	refY = data_loaded['ref'][1];
	print "refX: ",refX
	print "refY: ",refY
	resolution = data_loaded['resolution']
	width = data_loaded['width']
	height = data_loaded['height']
	filename = data_loaded['filename']
	time = data_loaded['startTime']
	mapTorviz = data_loaded['mapTorviz']





if __name__ == '__main__':
	rospy.init_node('water_current')
	
	config_file_name = sys.argv[1]
	parse_config_file(config_file_name)
	try:
		print "Abrindo arquivo..."
      		fid=h5py.h5f.open(filename,flags=h5py.h5f.ACC_RDONLY)
	except IOError as e:
		sys.stderr.write('Unable to open File: '+filename+'\n')
		print(e.errno)
		print(e)
		exit()
	except:
    		print "Unexpected error:", sys.exc_info()[0]
		raise
    	else:
      		print "Fim"



	t=type(fid)
	if t==h5py.h5f.FileID:
		txt=os.path.basename(fid.name)
	elif t==h5py.h5g.GroupID:
		txt=fid.name
	else:
		txt='root'
	
	print "root: ",txt

	level =0
	PrintHierarchy(fid, level, ">")


	print "datasetX: ",len(datasetX), ", ", len(datasetX[0])
	print "datasetY: ",len(datasetY), ", ", len(datasetY[0])
	print "datasetCoord: ",len(datasetCoord), ", ", len(datasetCoord[0])
	print "time interval: 0 - ",(len(datasetX)-1)
	minX =datasetCoord[0][0]
	maxX =datasetCoord[0][0]
	minY =datasetCoord[0][1]
	maxY =datasetCoord[0][1]
	startRosService();
	print "mapTorviz: ",mapTorviz
	if (mapTorviz == 1):
		loadMap()



	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		try:
			if (mapTorviz == 1):
				rospy.logerr("---sending map! Time: %d", time)
				pub.publish(mymap);
				rospy.logerr("---map sent!")
			rate.sleep()
			rospy.logerr("---end sleeping")
		except rospy.ROSInterruptException:
			rospy.logerr("ROS InterruptException! Just ignore the exception!") 
		except rospy.ROSTimeMovedBackwardsException:
			rospy.logerr("ROS Time Backwards! Just ignore the exception!")

