
import rosbag
from sklearn.metrics import mean_squared_error
from math import sqrt
import numpy
import time as tempo
import os
import locale

locale.setlocale(locale.LC_ALL, "")

oldNtime =0
oldStime =0
y_actual = []
y_predicted = []
time = []
posY = []
posRefX = []
posRefY = []
simNumber = 0
#meanTime, stdTime, meanDist, stdDist, meanError, stdError
referenceTime = 0
referenceTravelDistance = 0.0
prefix = "diff"
scenario = "1"
disturbType = "medio"

for topic, msg, t in rosbag.Bag('./bags2/'+prefix+'boat_scenario'+scenario+'_noDisturbs.bag').read_messages():
	timeNow = int(msg.header.stamp.secs*100+msg.header.stamp.nsecs/10000000)
	referenceTime = float(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)

	if (timeNow >= len(posRefX)):	
		posRefX.append(msg.pose.pose.position.x)
		posRefY.append(msg.pose.pose.position.y)
		print timeNow,"/",len(posRefX), msg.pose.pose.position.x, msg.pose.pose.position.y
	else:
		posRefX[timeNow] = msg.pose.pose.position.x
		posRefY[timeNow] = msg.pose.pose.position.y
#	posRefY[timeNow] = msg.pose.pose.position.y


refFile = open("bags2/processado/referencia", "w")
for t in range(0, len(posRefX)):
	refFile.write(str(t))
	refFile.write(";")
	refFile.write(locale.format("%1.2f",posRefX[t],0) )
	refFile.write(";")
	refFile.write(locale.format("%1.2f",posRefY[t],0) )
	refFile.write(";\n")
refFile.close()


for i in range(1, len(posRefY)):
	delta = sqrt((posRefX[i]*100-posRefX[i-1]*100)*(posRefX[i]*100-posRefX[i-1]*100)+(posRefY[i]*100-posRefY[i-1]*100)*(posRefY[i]*100-posRefY[i-1]*100))
	referenceTravelDistance += delta
#for x in range(0, 27500):
#	posRef.extend([95])

posX = []
posX.append([])
posY = []
posY.append([])
print posX
print posY
dist = [0]


oldX =0
oldY =0
first=True
for topic, msg, t in rosbag.Bag('./bags2/'+prefix+'boat_scenario'+scenario+disturbType+'.bag').read_messages():

#	print int(msg.pose.pose.position.x*100), ", ",msg.pose.pose.position.y
	#posY[simNumber][int(msg.pose.pose.position.x*100)] = msg.pose.pose.position.y

	if (oldStime > msg.header.stamp.secs): 
		time.extend( [float(oldStime + oldNtime/1000000000.0)] )
		simNumber+=1
		first=True
		print "################################################################################################"
		print "################################################################################################"
		print "\nDetected end of Simulation at ", oldStime,".",oldNtime,". New simulation started at ", msg.header.stamp.secs,".",msg.header.stamp.nsecs
		print "time: ",time
		print "\n"
		posX.append([])
		posY.append([])
		dist.append(0)
	else:
		if (msg.header.stamp.secs-oldStime > 1):
			continue;
		if (first):
			first = False
		else:
			delta = sqrt((msg.pose.pose.position.x*100-oldX)*(msg.pose.pose.position.x*100-oldX)+(msg.pose.pose.position.y*100-oldY)*(msg.pose.pose.position.y*100-oldY))
			#### remove publications after restart... No robot will move 1 cm in that time
			if (delta > 100):
				print "Delta: ",delta,"pos: ",msg.pose.pose.position.x*100, msg.pose.pose.position.y*100," old: ",oldX, oldY
				print msg.header.stamp.secs,msg.header.stamp.nsecs
				tempo.sleep(1)
			else:
				dist[simNumber]=dist[simNumber]+delta
		timeNow = int(msg.header.stamp.secs*100+msg.header.stamp.nsecs/10000000)
		if (timeNow >= len(posX[simNumber])):	
			posX[simNumber].append(msg.pose.pose.position.x)
			posY[simNumber].append(msg.pose.pose.position.y)
		if (msg.header.stamp.secs-oldStime == 1):
			print "dist[",simNumber,"]: ",dist, oldX, oldY, " time: ", msg.header.stamp.secs, oldStime

		oldX= (msg.pose.pose.position.x*100)
		oldY= (msg.pose.pose.position.y*100)
	oldNtime = msg.header.stamp.nsecs
	oldStime = msg.header.stamp.secs

time.extend( [float(oldStime + oldNtime/1000000000.0)] )
simNumber+=1

print "time: ",time
print "time: ",dist
print "########### RESULT ###########"
print "Number of Simulations:", simNumber
print "Reference boat - total time: ",referenceTime
print "Reference boat - traveled distance: ",referenceTravelDistance
meanTime = numpy.mean(time, axis=0)
stdTime = numpy.std(time, axis=0)
print "meanTime: ",meanTime," - ",time
print "stdTime: ",stdTime
meanDist = numpy.mean(dist, axis=0)
stdDist = numpy.std(dist, axis=0)
print "meanDist: ",meanDist," - ",dist
print "stdDist: ",stdDist
print " len posRefX: ",len(posRefX)
print " len posRefY: ",len(posRefY)
print " len posX: ",len(posX)
print " len posY: ",len(posY)
preparedRefX = []
preparedRefY = []
meanErrorX = []
meanErrorY = []
meanError = []
stdError = []
distance = []
for n in range(0, simNumber):
	print "-----------Simulation ",n
	print " len posY: ",len(posX[n])
	print " len posY: ",len(posY[n])
	preparedRefX.append([])
	preparedRefY.append([])
	distance.append([])
	for t in range(0, len(posX[n])):
		if (t < len(posRefX)):
			preparedRefX[n].append(posRefX[t])		
			preparedRefY[n].append(posRefY[t])		
		else:
			preparedRefX[n].append(posRefX[len(posRefX)-1])
			preparedRefY[n].append(posRefY[len(posRefY)-1])
		distance[n].append(  sqrt( (posX[n][t]-preparedRefX[n][t])*(posX[n][t]-preparedRefX[n][t])+ (posY[n][t]-preparedRefY[n][t])*(posY[n][t]-preparedRefY[n][t]) ) )
	meanErrorX.append( sqrt(mean_squared_error(posX[n], preparedRefX[n])) )
	meanErrorY.append( sqrt(mean_squared_error(posY[n], preparedRefY[n])) )
	meanError.append( numpy.mean(distance[n], axis=0) )
	stdError.append( numpy.std(distance[n], axis=0) )
	print "meanError[",n,"]: x: ",meanErrorX[n]," y: ", meanErrorY[n]
	print "meanError[",n,"]: ",meanError[n]," std: ", meanErrorY[n]
	#print "stdError[",n,"]: ",stdError
meanMeanErrorX = numpy.mean(meanErrorX, axis=0)
stdMeanErrorX = numpy.std(meanErrorX, axis=0)
meanMeanErrorY = numpy.mean(meanErrorY, axis=0)
stdMeanErrorY = numpy.std(meanErrorY, axis=0)
meanMeanError = numpy.mean(meanError, axis=0)
stdMeanError  = numpy.std(meanError, axis=0)
#print "meanMeanErrorX: ", meanMeanErrorX, " stdX: ", stdMeanErrorX
#print "meanMeanErrorY: ", meanMeanErrorY, " stdY: ", stdMeanErrorY
#print "final meanError: ", meanMeanError, " std: ", stdMeanError

print "---- PREPARING OUTPUT DATA FILE ---"


#for n in range(0, simNumber):
	#print "preparedRefX[",n,"]: ",len(preparedRefX[n]), len(preparedRefY[n]), "pos : ", len(posX[n]), len(posY[n])
output = []
for n in range(0, simNumber):
	output.append([])
	for t in range(0, len(preparedRefX[n])):
		if (t ==0):
			output[n].append(t)
			output[n].append(preparedRefX[n][t])
			output[n].append(preparedRefY[n][t])
		output[n].append(posX[n][t])
		output[n].append(posY[n][t])

texto = []
#for l in range(0, len(output)):
#	linha = " "
#	for c in range(0, len(output[l])):
#		linha += str(output[l][c])+";"
#	texto.append(linha+"\n")
#	print linha
print len(texto)
for n in range(0, simNumber):
	outFile = open("bags2/processado/output"+prefix+scenario+disturbType+"sim"+str(n)+".txt","w") 
	for t in range(0, len(posX[n])):
		outFile.write(str(t))
		outFile.write(";")
		outFile.write(locale.format("%1.2f",posX[n][t],0) )
		outFile.write(";")
		outFile.write(locale.format("%1.2f",posY[n][t],0) )
		outFile.write(";\n")
	outFile.close()


print "size posRef: ", len(posRefX), len(posRefY)
arrayTotalDist = []
for n in range(0, simNumber):
	print "----------simulation: ",n, " size pos: ", len(posX[n]), len(posY[n])
	totalDist = 0
	allDistances = []
	for t in range(0, len(posX[n])):
		deltaX = posX[n][t]-posRefX[t]
		deltaY = posY[n][t]-posRefY[t]
		delta = sqrt(deltaX*deltaX + deltaY*deltaY)
		allDistances.append(delta)
		totalDist += delta
	arrayTotalDist.append(totalDist/len(posX[n]))
	print ----------n," mean: ",numpy.mean(allDistances, axis=0)
	print ----------n," std: ", numpy.std(allDistances, axis=0)	
	print ----------n," media: ",arrayTotalDist[n]

meanMeanError = numpy.mean(arrayTotalDist, axis=0)
stdMeanError  = numpy.std(arrayTotalDist, axis=0)
print "final meanError: ", meanMeanError, " std: ", stdMeanError
