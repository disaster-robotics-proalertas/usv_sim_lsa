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

originX = 0
originY = 0
resolution = 0.5
obstacle_image = "no.jpg"

pub = rospy.Publisher('windflow', OccupancyGrid, queue_size=1)
mymap = OccupancyGrid();
minHeight= 0
maxHeight= 0
minColisionHeight = 0
maxColisionHeight = 0
windSpeed = 0


# Define constants:
height = 10				# lattice dimensions
width = 10
viscosity = 0.02			# fluid viscosity
omega = 1 / (50*viscosity + 0.5)	# "relaxation" parameter
u0 = 1				# initial and in-flow speed
four9ths = 4.0/9.0			# abbreviations for lattice-Boltzmann weight factors
one9th   = 1.0/9.0
one36th  = 1.0/36.0
performanceData = False			# set to True if performance data is desired
barrier = []

rho=[]
ux=[]
uy=[]
n0=[]
nN=[]
nS=[]
nE=[]
nW=[]
nNE=[]
nNW=[]
nSE=[]
nSW=[]
theFig = 0
fluidImage = 0
bImageArray= 0
barrierImage=0
def initilizeArray():
	global width, height, resolution
	global rho, ux, uy, n0, nN, nS, nE, nW, nNE, nNW, nSE, nSW, u0
	global theFig, fluidImage, bImageArray, barrierImage, barrier
	u0 = u0 *resolution
	# Initialize all the arrays to steady rightward flow:
	n0 = four9ths * (numpy.ones((height,width)) - 1.5*u0**2)	# particle densities along 9 directions
	nN = one9th * (numpy.ones((height,width)) - 1.5*u0**2)
	nS = one9th * (numpy.ones((height,width)) - 1.5*u0**2)
	nE = one9th * (numpy.ones((height,width)) + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nW = one9th * (numpy.ones((height,width)) - 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nNE = one36th * (numpy.ones((height,width)) + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nSE = one36th * (numpy.ones((height,width)) + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nNW = one36th * (numpy.ones((height,width)) - 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nSW = one36th * (numpy.ones((height,width)) - 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	rho = n0 + nN + nS + nE + nW + nNE + nSE + nNW + nSW		# macroscopic density
	ux = (nE + nNE + nSE - nW - nNW - nSW) / rho			# macroscopic x velocity
	uy = (nN + nNE + nNW - nS - nSE - nSW) / rho			# macroscopic y velocity


	# Here comes the graphics and animation...
	theFig = matplotlib.pyplot.figure(figsize=(8,3))
	fluidImage = matplotlib.pyplot.imshow(
						curl(ux, uy), 
						origin='upper', 
						norm=matplotlib.pyplot.Normalize(-.1,.1), 
						cmap=matplotlib.pyplot.get_cmap('cool'), interpolation='none')
			# See http://www.loria.fr/~rougier/teaching/matplotlib/#colormaps for other cmap options
	bImageArray = numpy.zeros((height, width, 4), numpy.uint8)	# an RGBA image
	bImageArray[barrier,3] = 255								# set alpha=255 only at barrier sites
	barrierImage = matplotlib.pyplot.imshow(bImageArray, origin='upper', interpolation='none')

	

# Move all particles by one step along their directions of motion (pbc):
def stream():
	global nN, nS, nE, nW, nNE, nNW, nSE, nSW, desX, desY,posX, posY
	global barrier
	nN  = numpy.roll(nN,   1, axis=0)	# axis 0 is north-south; + direction is north
	nNE = numpy.roll(nNE,  1, axis=0)
	nNW = numpy.roll(nNW,  1, axis=0)
	nS  = numpy.roll(nS,  -1, axis=0)
	nSE = numpy.roll(nSE, -1, axis=0)
	nSW = numpy.roll(nSW, -1, axis=0)
	nE  = numpy.roll(nE,   1, axis=1)	# axis 1 is east-west; + direction is east
	nNE = numpy.roll(nNE,  1, axis=1)
	nSE = numpy.roll(nSE,  1, axis=1)
	nW  = numpy.roll(nW,  -1, axis=1)
	nNW = numpy.roll(nNW, -1, axis=1)
	nSW = numpy.roll(nSW, -1, axis=1)

	barrierN = numpy.roll(barrier,  1, axis=0)				# sites just north of barriers
	barrierS = numpy.roll(barrier, -1, axis=0)				# sites just south of barriers
	barrierE = numpy.roll(barrier,  1, axis=1)				# etc.
	barrierW = numpy.roll(barrier, -1, axis=1)
	barrierNE = numpy.roll(barrierN,  1, axis=1)
	barrierNW = numpy.roll(barrierN, -1, axis=1)
	barrierSE = numpy.roll(barrierS,  1, axis=1)
	barrierSW = numpy.roll(barrierS, -1, axis=1)
	# Use tricky boolean arrays to handle barrier collisions (bounce-back):
	nN[barrierN] = nS[barrier]
	nS[barrierS] = nN[barrier]
	nE[barrierE] = nW[barrier]
	nW[barrierW] = nE[barrier]
	nNE[barrierNE] = nSW[barrier]
	nNW[barrierNW] = nSE[barrier]
	nSE[barrierSE] = nNW[barrier]
	nSW[barrierSW] = nNE[barrier]
        #print("------------------------------")
        #print nS
	#print barrier
        #print nS[barrier]
		
# Collide particles within each cell to redistribute velocities (could be optimized a little more):
def collide():
	global rho, ux, uy, n0, nN, nS, nE, nW, nNE, nNW, nSE, nSW
	rho = n0 + nN + nS + nE + nW + nNE + nSE + nNW + nSW
	ux = (nE + nNE + nSE - nW - nNW - nSW) / rho
	uy = (nN + nNE + nNW - nS - nSE - nSW) / rho
	ux2 = ux * ux				# pre-compute terms used repeatedly...
	uy2 = uy * uy
	u2 = ux2 + uy2
	omu215 = 1 - 1.5*u2			# "one minus u2 times 1.5"
	uxuy = ux * uy
	n0 = (1-omega)*n0 + omega * four9ths * rho * omu215

	nN = (1-omega)*nN + omega * one9th * rho * (omu215 + 3*uy + 4.5*uy2)
	nS = (1-omega)*nS + omega * one9th * rho * (omu215 - 3*uy + 4.5*uy2)
	nE = (1-omega)*nE + omega * one9th * rho * (omu215 + 3*ux + 4.5*ux2)
	nW = (1-omega)*nW + omega * one9th * rho * (omu215 - 3*ux + 4.5*ux2)
	nNE = (1-omega)*nNE + omega * one36th * rho * (omu215 + 3*(ux+uy) + 4.5*(u2+2*uxuy))
	nNW = (1-omega)*nNW + omega * one36th * rho * (omu215 + 3*(-ux+uy) + 4.5*(u2-2*uxuy))
	nSE = (1-omega)*nSE + omega * one36th * rho * (omu215 + 3*(ux-uy) + 4.5*(u2-2*uxuy))
	nSW = (1-omega)*nSW + omega * one36th * rho * (omu215 + 3*(-ux-uy) + 4.5*(u2+2*uxuy))
	# Force steady rightward flow at ends (no need to set 0, N, and S components):# Define constants:
	nE[:,0] = one9th * (1 + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	#print one9th
	nW[:,0] = one9th * (1 - 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nNE[:,0] = one36th * (1 + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nSE[:,0] = one36th * (1 + 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nNW[:,0] = one36th * (1 - 3*u0 + 4.5*u0**2 - 1.5*u0**2)
	nSW[:,0] = one36th * (1 - 3*u0 + 4.5*u0**2 - 1.5*u0**2)

# Compute curl of the macroscopic velocity field:
def curl(ux, uy):
	return numpy.roll(uy,-1,axis=1) - numpy.roll(uy,1,axis=1) - numpy.roll(ux,-1,axis=0) + numpy.roll(ux,1,axis=0)

def curlX(ux):
	return - numpy.roll(ux,-1,axis=0) + numpy.roll(ux,1,axis=0)

def curlY(uy):
	return numpy.roll(uy,-1,axis=1) - numpy.roll(uy,1,axis=1)




atualiza=0

# Function called for each suc


def nextFrame(arg):				# (arg is the frame number, which we don't need)
	global startTime
	global atualiza, biggestSpeed,smallestSpeed
	global barrier, barrierImage
	global resolution, originX, originY, mymap
	if performanceData and (arg%100 == 0) and (arg > 0):
		endTime = time.clock()
		print "%1.1f" % (100/(endTime-startTime)), 'frames per second'
		startTime = endTime
	frameName = "frame%04d.png" % arg
	#matplotlib.pyplot.savefig(frameName)
	#frameList.write(frameName + '\n')
	#print arg



	for step in range(2):					# adjust number of steps for smooth animation
		stream()
		collide()
	#print ("ux",ux)
	fluidImage.set_array(curl(ux, uy))
	atualiza=atualiza+1
	if (atualiza == 20):
		atualiza = 0
		#bImageArray = numpy.zeros((height, width, 4), numpy.uint8)
		#bImageArray[barrier,3] = 255
		#barrierImage = matplotlib.pyplot.imshow(bImageArray, origin='lower', interpolation='none')
		
#	fluidImage.set_array(curlX(ux))
#	fluidImage.set_array(curlY(uy))

	myarray = curl(ux, uy);

		
	return (fluidImage, barrierImage)		# return the figure elements to redraw
	


def handleWindCurrent(req):
	global originX, originY, ux, uy, windSpeed, resolution
	global barrierN, barrierS, barrierE, barrierW, barrierNE, barrierNW, barrierSE, barrierSW, barrier
	print ("\n Received request",req.x,", ",req.y)
	x = (req.x-originX)*resolution
	if (x < 0):
		x = 0
	if (x >= width):
		x = width-1
	y = (req.y-originY)*resolution
	if (y < 0):
		y = 0
	if (y >= height):
		y = height-1
	
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
	print "\n[",x,",",y,"]: (",ux[x][y]/resolution,",",uy[x][y]/resolution,")"
	return  GetSpeedResponse(ux[x][y]/resolution, uy[x][y]/resolution)
#        return GetSpeedResponse(ux[req.x][req.y], uy[req.x][req.y])


def parse_config_file(config_file_name):
	global resolution, windSpeed
	global originX
	global originY
	global minColisionHeight, maxColisionHeight
	global width, height, dataset, array
	global heightMapFile, obstacle_image, u0
	with open(config_file_name, 'r') as stream:
        	data_loaded = yaml.load(stream)
	print ('--------------------- Loading yaml file ', config_file_name)
	originX = data_loaded['origin'][0];
	originY = data_loaded['origin'][1];
	resolution = data_loaded['resolution']
	windSpeed = data_loaded['windSpeed']
	u0 = data_loaded['windSpeed']
	obstacle_image = data_loaded['obstacleImage']
	minColisionHeight = data_loaded['autodraw']['minColisionHeight']
	maxColisionHeight = data_loaded['autodraw']['maxColisionHeight']
	heightMapFile = data_loaded['heightMap']
	print (data_loaded['origin'])
	print ("originx ",data_loaded['origin'][0])
	print ("originy ",data_loaded['origin'][1])
	print (heightMapFile)
	print (data_loaded['resolution'])


def loadColisions():
	global obstacle_image, heightMapFile, array, width, height, minHeight, maxHeight, minColisionHeight, maxColisionHeight
	global barrierN, barrierS, barrierE, barrierW, barrierNE, barrierNW, barrierSE, barrierSW, barrier
	# loading height map image
	dataset = gdal.Open(heightMapFile, gdal.GA_ReadOnly)
	try:
		srcband = dataset.GetRasterBand(1)
	except RuntimeError, e:
        	print 'No band %i found' % band_num
        	print e
	        sys.exit(1)

	print "raster count: ",dataset.RasterCount
	for x in range(1, dataset.RasterCount + 1):
		band = dataset.GetRasterBand(x)
	    	array = band.ReadAsArray()
		minHeight = band.GetMinimum()
		maxHeight = band.GetMaximum()

	minHeight = srcband.GetMinimum()
	maxHeight = srcband.GetMaximum()

	if (minHeight == None) or (maxHeight == None):
		print "Calculating min and max speed!"
		print len(array)
		minHeight = array[0][0]
		maxHeight = array[0][0]
		for i in range(0, len(array)):
			for j in range(0, len(array[i])):
				if (array[i][j] < minHeight):
					minHeight = array[i][j]
				if (array[i][j] > maxHeight):
					maxHeight = array[i][j]
	print "min: ", minHeight
	print "max: ", maxHeight
	print "scale: ", srcband.GetScale()
	print "unit_type: ", srcband.GetUnitType()

	obstaculos = scipy.misc.imread(obstacle_image, True)
	if (len(obstaculos) == len(array)) and (len(obstaculos[0]) == len(array[0])):
		print "Tamanhos identicos!"
		height = (int)(len(obstaculos)*resolution)
		width  = (int)(len(obstaculos[0])*resolution)
		print obstacle_image,": (",len(obstaculos),", ",len(obstaculos[0]),")"
	else:
		print "tamanhos das imagens diferentes!"
		print heightMapFile,": (",len(array),", ",len(array[0]),")"
		print obstacle_image,": (",len(obstaculos),", ",len(obstaculos[0]),")"
		exit(0)
	barrier = numpy.zeros((height,width), bool)					# True wherever there's a barrier
	print "height: ",height
	print "width: ",width
	data = numpy.zeros((len(obstaculos), len(obstaculos[0]), 3), dtype=numpy.uint8)
	data[:] = [255,255,255]
	for y in range(0,len(obstaculos)):
		for x in range(0,len(obstaculos[y])):
			scaledX = int(math.floor(x*resolution))
			if (scaledX >= len(barrier[0])):
				scaledX = len(barrier[0])-1
			scaledY = int(math.floor(y*resolution))
			if (scaledY >= len(barrier)):
				scaledY = len(barrier)-1
			if (obstaculos[y][x] < 100):
				barrier[scaledY,scaledX]=True
				data[y][x]=[0,0,0]
			if (array[y][x] >= minColisionHeight) and (array[y][x] <= maxColisionHeight):
				barrier[scaledY,scaledX]=True
				data[y][x]=[0,0,0]
	
	img = Image.fromarray(data, 'RGB')
	img.save('combinedObstacle.png')

	barrierN = numpy.roll(barrier,  1, axis=0)				# sites just north of barriers
	barrierS = numpy.roll(barrier, -1, axis=0)				# sites just south of barriers
	barrierE = numpy.roll(barrier,  1, axis=1)				# etc.
	barrierW = numpy.roll(barrier, -1, axis=1)
	barrierNE = numpy.roll(barrierN,  1, axis=1)
	barrierNW = numpy.roll(barrierN, -1, axis=1)
	barrierSE = numpy.roll(barrierS,  1, axis=1)
	barrierSW = numpy.roll(barrierS, -1, axis=1)

	



def startRosService():
        s = rospy.Service('windCurrent', GetSpeed, handleWindCurrent)
        print "Ready to answer wind current."

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
	rospy.loginfo ("################### Preparing map. Size (%d, %d) origin(%d, %d)", width, height, originX, originY)
	for x in xrange(height-1, -1, -1):
		for y in range(0, width):
			mymap.data.append(0);			
	rospy.loginfo("#################### Map loaded!")

if __name__ == '__main__':
	rospy.init_node('wind_current')
	rospy.loginfo ("starting wind_current. argument count: %d", len(sys.argv))
	if (len(sys.argv)!=2 and len(sys.argv)!=4):
		rospy.logerr ("###################Error: argument count invalid! %d", len(sys.argv))
		for i in range(0, len(sys.argv)):
			rospy.logerr(" argv %d : %s", i, sys.argv[i])
		exit(0)

	config_file_name = sys.argv[1]
	parse_config_file(config_file_name)
	loadColisions()
	initilizeArray()
	startRosService();
	loadMap()
	rate = rospy.Rate(30) # 10hz
	interval = 0
	animate = matplotlib.animation.FuncAnimation(theFig, nextFrame, interval=1, blit=True)
	matplotlib.pyplot.show()
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


	
