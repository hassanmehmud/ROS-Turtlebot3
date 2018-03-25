#!/usr/bin/env python
from node import node
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import math
# neighbour = []
class goToPose():
	#topic information and algorithm
	#neighbour = []
    def __init__(self):
    	rospy.init_node('mynode')
        rospy.wait_for_message('/map',OccupancyGrid)
        rospy.Subscriber('/map',OccupancyGrid,self.getMap)
        rospy.spin()
#map information and conversion (grid coordinates)
    def getMap(self,dataMsg):
    	emptyList = [] 
    	idGlobal = 0
    	mapDataCounter = 0
    	self.data = dataMsg.data
    	#find me (map coordinates)
    	for i in range(-10,374):
    		for j in range(-10,374):
				newNode = node(0,0,0,0,0,0,0) 
				newNode.ids = idGlobal                                                                                                                                              
				newNode.status = self.data[mapDataCounter]
				newNode.x = j
				newNode.y = i
				
				emptyList.append(newNode)
				mapDataCounter = mapDataCounter+1
				idGlobal = idGlobal+1
		#print len(emptyList)
	startNode = emptyList[407]
	print startNode.x
	print startNode.y
# help me search my neighbours
	goalNode = emptyList[89]
	neighbour = []
	newNode1 = node(0,0,0,0,0,0,0) 
	currentNode = startNode
	for x in emptyList:
		
		if (x.x == (currentNode.x-1) and x.y == currentNode.y):
			newNode1 = x
			neighbour.append(newNode1)

		if (x.x == (currentNode.x+1) and x.y == currentNode.y):
			newNode1 = x
			neighbour.append(newNode1)

		if (x.x == (currentNode.x) and x.y == currentNode.y+1):
			newNode1 = x
			neighbour.append(newNode1)

		if (x.x == (currentNode.x) and x.y == currentNode.y-1):
			newNode1 = x 
			print newNode1.x
			print newNode1.y
			neighbour.append(newNode1)

		if (x.x == (currentNode.x+1) and x.y == currentNode.y+1):
			newNode1 = x
			print newNode1.x
			print newNode1.y
			neighbour.append(newNode1)

		if (x.x == (currentNode.x-1) and x.y == currentNode.y+1):
			newNode1 = x
			neighbour.append(newNode1)

		if (x.x == (currentNode.x+1) and x.y == currentNode.y-1):
			newNode1 = x
			neighbour.append(newNode1)

		if (x.x == (currentNode.x-1) and x.y == currentNode.y-1):
			newNode1 = x
			neighbour.append(newNode1)
			print str(len(neighbour)) + 'inin'

	print str(len(neighbour)) +'xyz'
	o=0
	def heuristicsCostEst(self, xDest, yDest):
#Euclidean for intelligent distance guess	
	    xd = xDest - self.x
	    yd = yDest - self.y
	    print xd + yd
	    d = math.sqrt(xd * xd + yd * yd)
	    return (d)

	movementCost = [10, 10, 10, 10, 14, 14, 14, 14 ]
	print str(neighbour) + 'abc'
	for val in neighbour:
		val.g = currentNode.g + movementCost[o]
		val.h = heuristicsCostEst(val,60,34)
		val.f = val.g + val.h
		val.parent = currentNode.ids
		o = o + 1
	print neighbour[4].x
	print neighbour[4].h

		
        




if __name__ == '__main__':

    try:
        goToPose()
    except:
    	rospy.loginfo("Terminated")

