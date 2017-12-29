#!/usr/bin/env python

import numpy, time, matplotlib.pyplot, matplotlib.animation
from PIL import Image
from scipy import misc
from scipy import ndimage
import yaml
import math
import scipy
import rospy

from usv_water_current.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion

from PIL import Image
import matplotlib.pyplot as plt
from osgeo import gdal


pub = rospy.Publisher('waterflow', OccupancyGrid, queue_size=1)

def handleWaterCurrent(req):
	global originX, originY, array
#	print ("\n Received request",req.x,", ",req.y)
	x = req.x-originX
	if (x < 0):
		x = 0
	if (x >= height):
		x = height-1
	y = req.y-originY
	if (y < 0):
		y = 0
	if (y >= width):
		y = width-1
	print "\n[",x,",",y,"]: (",array[x][y],",",array[x][y],")"
	return  GetSpeedResponse(array[x][y], array[x][y])
#        return GetSpeedResponse(ux[req.x][req.y], uy[req.x][req.y])


def parse_config_file(config_file_name):
	global resolution
	global originX
	global originY
	global width, height, dataset, array
	with open(config_file_name, 'r') as stream:
        	data_loaded = yaml.load(stream)
	print ('--------------------- Loading yaml file ', config_file_name)
	originX = data_loaded['origin'][0];
	originY = data_loaded['origin'][1];
	resolution = data_loaded['resolution']
	width = data_loaded['simulation']['width']
	height = data_loaded['simulation']['height']
	print (data_loaded['origin'])
	print (data_loaded['origin'][0])
	print (data_loaded['origin'][1])
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
	print "min: ", srcband.GetMinimum()
	print "max: ", srcband.GetMaximum()
	print "scale: ", srcband.GetScale()
	print "unit_type: ", srcband.GetUnitType()

def startRosService():
        s = rospy.Service('waterCurrent', GetSpeed, handleWaterCurrent)
        print "Ready to answer water current."

def publishMap():
	global array, width, height, originX, originY
	mynorm = matplotlib.colors.Normalize(vmin=-0.1,vmax=0.1);

	mymap = OccupancyGrid();
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
	print "Preparing map"
	for x in range(0,height):
		for y in range(0,width):
			value = 127*mynorm(array[x][y]);
#	
			if (value >127):
				value = 127;
			elif (value < 0):
				value = 0;
			mymap.data.append(int(value));			
	print "Sending map"
	pub.publish(mymap);

if __name__ == '__main__':
	print "Hello!"
	rospy.init_node('usv_water_current')
	config_file_name = sys.argv[1]
	parse_config_file(config_file_name)
	startRosService();
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		publishMap()
		rate.sleep()



	
