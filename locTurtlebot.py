#!/usr/bin/env python
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
#TODO requires bit of alteration
landmarks = []
landmarks.append(("Coffee shop", 0.91, -0.95));
landmarks.append(("Coffee shop", 0.10, -0.404));
landmarks.append(("Coffee shop", -1.14, -2.88));
landmarks.append(("Coffee shop", -2.59, -0.83));
landmarks.append(("Coffee shop", -2.59, 0.53));
landmarks.append(("Coffee shop", 1.29, -1.09));
#calculate distance estimate
def distEstimate(x1, y1, x2, y2):
    xD = x1 - x2
    yD = y1 - y2
    return math.sqrt(xD*xD + yD*yD)
#call back function for retrieving position from osm
def callBack(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    nearTo = None
    nearestDist = None
    for locName, loc_x, loc_y, in landmarks:
        lDist = distEstimate(x, y, loc_x, loc_y)
        if nearestDist is None or lDist < nearestDist:
            nearTo = locName
            nearestDist = lDist
            #format values
    rospy.loginfo('x: {}, y: {}'.format(x, y))
#
def main():
    #rospy.wait_for_message ('move_base_simple/goal', PoseStamped)
    #rospy.Subscriber('move_base_simwple/goal', PoseStamped)
    rospy.init_node("Location_Retriever")
    rospy.Subscriber("odom",Odometry, callBack)
    rospy.spin()

if __name__ == '__main__':
    main()

