#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from node import node
import copy
import numpy as np
import math

class goToPose():
	#topic information and algorithm
	def __init__(self):
		rospy.init_node("rosNode")
		rospy.wait_for_message('/map', OccupancyGrid)
		rospy.Subscriber('/map',OccupancyGrid,self.map)
		rospy.spin()
	#map information and conversion (grid coordinates)
	def map(self,mapMsg):
		emptyList = []
		globalId = 0
		mapDataCounter = 0
		self.data = mapMsg.data
		#find me (map coordinates)
		for i in range(-10,374):
			for j in range(-10,374):
				newNode = node(0,0,0,0,0,0,0,0)
				newNode.ids = globalId
				newNode.status = self.data[mapDataCounter]
				newNode.x = j
				newNode.y = i
				emptyList.append(newNode)
				mapDataCounter = mapDataCounter + 1
				globalId = globalId + 1
	# help me search my neighbours
		startNode = emptyList[407]
		goalNode = emptyList[89]
		neighbour = []
		newNode1 = node(0,0,0,0,0,0,0,0)
		currentNode = startNode
		#TODO Loop
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
				neighbour.append(newNode1)
				print newNode1.x
				print newNode1.y
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
		print len(neighbour)
		#TODO
		o = 0
		#TODO --> Euclidean for intelligent distance guess
	def heuristicsCostEst(self, xDest, yDest):
		xD = xDest - self.x
		yD = yDest - self.y
		d = math.sqrt(xD * xD + yD * yD)
		#return (d)
		movCost= [10, 10, 10, 10, 14, 14, 14, 14 ]
		for val in neighbour:
			val.g = currentNode.g + movCost[o]
			val.h = heuristicsCostEst(val,60,34)
			val.f = val.g + val.h
			val.parent = currentNode.ids
			o = o + 1
		print o
		print neighbour[4].x
		print neighbour[4].h
	

if __name__ == '__main__':

    try:
        goToPose()
    except:
    	rospy.loginfo("Terminated")

