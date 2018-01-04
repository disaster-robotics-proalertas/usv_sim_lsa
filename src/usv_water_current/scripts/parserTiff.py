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

from usv_water_current.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

from PIL import Image
import matplotlib.pyplot as plt
from osgeo import gdal
import struct


pub = rospy.Publisher('waterflow', OccupancyGrid, queue_size=1)
mymap = OccupancyGrid();
minSpeed= 0
maxSpeed= 0

def handleWaterCurrent(req):
	global originX, originY, array
#	print ("\n Received request",req.x,", ",req.y)

	x = req.x-originX
	if (x < 0):
		x = 0
	if (x >= height):
		x = height-1
	y = height-req.y-originY
	if (y < 0):
		y = 0
	if (y >= width):
		y = width-1
	rospy.logerr ("\n Received request %d, %d - (%d, %d) = %f",req.x,req.y,x,y,array[x][y])
	print "\n[",x,",",y,"]: (",array[x][y],",",array[x][y],")"
	mymap.data = []
	delta = maxSpeed - minSpeed
	for x2 in xrange(height-1, -1, -1):
		for y2 in range(0, width):

			value = 55*array[x2][y2]/delta;
			if (value >55):
				value = 55;
			elif (value < 0):
				value = 0;
			if (x==y2 and y==x2):
				print array[x2][y2]
				print "N: ",array[y+1][x]
				print "E: ",array[y][x-1]," C: ", array[y][x]," W: ",array[y][x+1]
				print "S: ",array[y-1][x]
				print array[y][x]
				value = 127
			mymap.data.append(int(value));			


	if (array[y][x] < minSpeed):
		return GetSpeedResponse(0, 0)
	return  GetSpeedResponse(array[y][x], array[y][x])
#        return GetSpeedResponse(ux[req.x][req.y], uy[req.x][req.y])


def parse_config_file(config_file_name):
	global resolution
	global originX
	global originY
	global width, height, dataset, array
	global minSpeed, maxSpeed
	with open(config_file_name, 'r') as stream:
        	data_loaded = yaml.load(stream)
	print ('--------------------- Loading yaml file ', config_file_name)
	originX = data_loaded['origin'][0];
	originY = data_loaded['origin'][1];
	resolution = data_loaded['resolution']
	width = data_loaded['simulation']['width']
	height = data_loaded['simulation']['height']
	print (data_loaded['origin'])
	print ("originx ",data_loaded['origin'][0])
	print ("originy ",data_loaded['origin'][1])
	print (data_loaded['image'])
	print (data_loaded['resolution'])
	dataset = gdal.Open(data_loaded['image'], gdal.GA_ReadOnly)
	try:
		srcband = dataset.GetRasterBand(1)
	except RuntimeError, e:
        	print 'No band %i found' % band_num
        	print e
	        sys.exit(1)

	for x in range(1, dataset.RasterCount + 1):
		band = dataset.GetRasterBand(x)
	    	array = band.ReadAsArray()
	minSpeed = srcband.GetMinimum()
	maxSpeed = srcband.GetMaximum()
	print "min: ", minSpeed
	print "max: ", maxSpeed
	print "scale: ", srcband.GetScale()
	print "unit_type: ", srcband.GetUnitType()
	geotransform = dataset.GetGeoTransform()
	if geotransform:
	    print("Origin = ({}, {})".format(geotransform[0], geotransform[3]))
	    print("Pixel Size = ({}, {})".format(geotransform[1], geotransform[5]))



def startRosService():
        s = rospy.Service('waterCurrent', GetSpeed, handleWaterCurrent)
        print "Ready to answer water current."

def loadMap():
	global array, width, height, originX, originY, mymap, minSpeed, maxSpeed
	mynorm = matplotlib.colors.Normalize(vmin=-0.1,vmax=0.1);

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
	delta = maxSpeed - minSpeed
	rospy.logerr ("################### Preparing map. Size (%d, %d) origin(%d, %d)", width, height, originX, originY)
	for x in xrange(height-1, -1, -1):
#	for x in range(0, height):
		for y in range(0, width):
			value = 127*array[x][y]/delta;
			if (value >127):
				value = 127;
			elif (value < 0):
				value = 0;
			mymap.data.append(int(value));			
	rospy.logerr("#################### Map loaded!")
	

if __name__ == '__main__':
	global mymap
	rospy.init_node('usv_water_current')
	rospy.logerr ("###################starting usv_water_current. argument count: %d", len(sys.argv))
	if (len(sys.argv)!=2 and len(sys.argv)!=4):
		rospy.logerr ("###################Error: argument count invalid! %d", len(sys.argv))
		for i in range(0, len(sys.argv)):
			rospy.logerr(" argv %d : %s", i, sys.argv[i])
		exit(0)
	config_file_name = sys.argv[1]
	parse_config_file(config_file_name)
	startRosService();
	loadMap()
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		rospy.logerr("---sending map! %d, %d", mymap.info.origin.position.x, mymap.info.origin.position.y)
		pub.publish(mymap);
		rospy.logerr("---map sent!")
		rate.sleep()
	print "Closing usv_water_current."


	
