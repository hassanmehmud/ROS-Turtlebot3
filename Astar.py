#!/usr/bin/env python
from node import node
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import math
import copy

class goToPose():
    global smallestScore
    global findNeighbours
    global checkOpenSet
    global calculateCosts
    global estimate
    global checkClosedSet
    global reConstruct
    #global closedSet
    global currentNode
    currentNode=node(-1,-1,-1,-1,-1,-1,-1,-1)
    global goalNode
    goalNode =node(-1,-1,-1,-1,-1,-1,-1,-1)
    global startNode
    startNode =node(-1,-1,-1,-1,-1,-1,-1,-1)
    #initializing rostopic data /map
    def __init__(self):
        global cameFrom
        cameFrom = []
        global openSet
        openSet = set()
        global closedSet
        closedSet = set()
        global neighbours
        neighbours=[]
        global obstStatus
        obstStatus = []
        rospy.init_node('mynode')
        '''rospy.Subscriber('odom' ,Odometry, self.update_odom)
    	print "asdasdasd"
        rospy.wait_for_message('/initialpose',PoseWithCovarianceStamped)
        rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,self.update_initial_pose)
        rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.update_goal)'''
        rospy.wait_for_message('/map',OccupancyGrid)
        rospy.Subscriber('/map',OccupancyGrid,self.updateMap)
        rospy.spin()
        '''def updateOdom(self,msg1):
    	self.x=msg1.pose.pose.position.x
    	self.y=msg1.pose.pose.position.y
    	grid_x = (int)((self.x + 10) / 0.05)
        grid_y = (int)((self.y + 10) / 0.05)
    	print grid_x
    	print grid_y'''
    '''def initPose(self,msg):
    	print "asdasdasd"
        self.x=msg.pose.pose.position.x
        self.x=msg.pose.pose.position.y
        print self.x
        grid_x = (int)((self.x + 10) / 0.05)
        grid_y = (int)((self.y + 10) / 0.05)
    	print grid_x
    	print grid_y
    def updateGoal(self,msg2):
    	self.z=msg2.pose.position.z
    	#print self.z'''
        # find out the smallest node in openset
    def smallestScore():

        try:
            test = next(iter(openSet))
            minimum=test
            for j in openSet:
                #print j.f
                if j.f < minimum.f:
                    minimum=j
            return minimum
        except StopIteration:
            print ("No more values to be iterated")
    # construct path from closedset objects
    def reConstruct(startN, goalN,ClosedSet):
        idsch=-1
        path=[]
        current1=goalN
        print goalN.parent
        print startN.parent
        print len(ClosedSet)
        path.append(goalN)
        while(current1.parent!=-1):
	        for x in ClosedSet:
	            if x.ids == current1.parent:
	                    path.append(x)
	                    current1=x
	                    idsch=x.ids
	                    break
            
        return path
        #calculate cost to estimate heuristics    
    def estimate(self, xDest, yDest):
        xD = xDest - self.x
        yD = yDest - self.y
        d = math.sqrt(xD * xD + yD * yD)
        return (d)
        #Euclidean theorem for intelligent distance guess
    def calculateCosts(currentNode):
        movementCost= [10, 10, 10, 10, 14, 14, 14, 14] 
        newNode=node(-1,-1,-1,-1,-1,-1,-1,-1) # new node initialized to append neighbours
        o=0
        for val in neighbours:
            val.g=currentNode.g + movementCost[o]
            val.h=estimate(val,goalNode.x,goalNode.y)
            val.f=val.g+val.h
            val.parent=currentNode.ids
            o=o+1
            # finding neighbours from map
    def findNeighbours(currentNode):
        # Clear neighbours list before iteration
        if neighbours:
            del neighbours[:]
        for x in cameFrom:

            if (x.x==(currentNode.x-1) and x.y==currentNode.y and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x+1) and x.y==currentNode.y and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x) and x.y==currentNode.y+1 and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x) and x.y==currentNode.y-1 and x.status!=100):
                newNode=x
                # prints found neighbours
                print str(newNode.x)+ ' / Neighbours'
                print str(newNode.y)+ ' / Neighbours'
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x+1) and x.y==currentNode.y+1 and x.status!=100):
                newNode=x
                # prints found neighbours
                print str(newNode.x) + ' / Neighbours'
                print str(newNode.y) + ' / Neighbours'
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)
                
            if (x.x==(currentNode.x-1) and x.y==currentNode.y+1 and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x+1) and x.y==currentNode.y-1 and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)

            if (x.x==(currentNode.x-1) and x.y==currentNode.y-1 and x.status!=100):
                newNode=x
                neighbours.append(newNode)
                fchStatus = x.status
                obstStatus.append(fchStatus)
        #print str(obstStatus) + '/Check status of obstacles on the path'
        #check open set items
    def checkOpenSet(x):
        i=x
        flag=True
        for j in openSet:
            if (i.ids==j.ids and i.f < j.f):
                j.g=i.g
                j.f=i.g+i.h
                j.parent=i.parent
                flag=False
                break
            if(i.ids==j.ids and i.f >= j.f):
                flag=False
                break
        if (flag==True):
            New2= copy.deepcopy(i)
            openSet.add(New2)
        return flag
        # check closed set items
    def checkClosedSet(x):
        i=x    
        flag1=True
        for j in closedSet:
            if (i.ids==j.ids):
                flag1=False
        return flag1

    def updateMap(self,mapDataMsg):
        idGlobal=0
        c=0
        # retrieve map data dynamically (changeset)
        self.data=mapDataMsg.data
        self.width=mapDataMsg.info.width
        self.height=mapDataMsg.info.height
        self.originX=int(mapDataMsg.info.origin.position.x)
        self.originY=int(mapDataMsg.info.origin.position.y)
        print str(self.originX) + '/ Origin X'
        print str(self.originY) + '/ Origin Y'
        self.widthNew =self.width + self.originX
        self.HeightNew =self.height + self.originY
        for i in range(self.originY,self.HeightNew):
            for j in range(self.originX,self.widthNew):
                n=node(-1,-1,-1,-1,-1,-1,-1,-1)
                n.ids=idGlobal
                n.status=self.data[c]
                n.x=j
                n.y=i
                cameFrom.append(n) # append map data to list
                c=c+1
                idGlobal=idGlobal+1
        goal=0
        goalNode=cameFrom[30]
        startNode=cameFrom[2]
        print str(startNode.x) + '/ Starting node X'
        print str (startNode.y) + '/ Starting node Y'
        print startNode.parent
        # check for obstacles i.e. start node and goal node is not an obstacle
        if(startNode.status != 100 and goalNode.status != 100):
            print str(startNode.status) + 'Obstacle status before add objects to sets Start Node'
            print str(goalNode.status) + 'Obstacle status before add objects to sets Goal Node'
            startNode.g=0
            xd = goalNode.x - startNode.x
            yd = goalNode.y - startNode.y
            startNode.h= math.sqrt(xd * xd + yd * yd)
            startNode.f=startNode.g + startNode.h
        else:
            print "Choose start and goal again"
        startNext = copy.deepcopy(startNode)
        openSet.add(startNext)
        #print list(openSet)
        currentNode=startNode
        # run, flow of algorithm
        while(goal ==0):
            if (len(openSet)!=0):
                smallest = smallestScore()
                currentNode=smallest
                Chosen = copy.deepcopy(smallest)
                openSet.remove(smallest)
                closedSet.add(Chosen)
                findNeighbours(currentNode)
                calculateCosts(currentNode)
                for x in neighbours:
                    if(goal ==1):
                        break
                    if (x.x == goalNode.x and x.y==goalNode.y):
                        goal=1
                        New = copy.deepcopy(x)
                        closedSet.add(New)
                    elif (checkClosedSet(x)):
                        checkOpenSet(x)

                    else:
                        continue
                      
        final_path=reConstruct(startNext, goalNode,closedSet)
        final_path.reverse()
        for c in final_path:
	        print '(' + str(c.x,)+',' + str(c.y,)+')',
		print "/ X,Y coordinates for found path"
if __name__ == '__main__':

    try:
        goToPose()
    except:
        rospy.loginfo("Terminated !")

