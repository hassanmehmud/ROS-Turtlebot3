#!/usr/bin/env python
#help for this code has been taken from Justin Huang 'jstnhuang'
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
class goalSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        pSeq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        #List of goal quaternions:
        qGoals = list()
        #List of goal poses:
        self.poseSeq = list()
        self.cGoal = 0
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            qGoals.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [pSeq[i:i+n] for i in range(0, len(pSeq), n)]
        rospy.loginfo(str(points))
        for point in points:
            #Exploit n variable to cycle in qGoals
            self.poseSeq.append(Pose(Point(*point),qGoals[n-3]))
            n += 1
        #rospy.loginfo(str(self.poseSeq))
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def activeCont(self):
        rospy.loginfo("Goal pose "+str(self.cGoal+1)+" is now being processed by the Action Server...")

    def respCB(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.cGoal)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.cGoal+1)+" received")

    def dControl(self, status, result):
        self.cGoal += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.cGoal)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.cGoal)+" reached") 
            if self.cGoal< len(self.poseSeq):
                nGoal = MoveBaseGoal()
                nGoal.target_pose.header.frame_id = "map"
                nGoal.target_pose.header.stamp = rospy.Time.now()
                nGoal.target_pose.pose = self.poseSeq[self.cGoal]
                rospy.loginfo("Sending goal pose "+str(self.cGoal+1)+" to Action Server")
                rospy.loginfo(str(self.poseSeq[self.cGoal]))
                self.client.send_goal(nGoal, self.dControl, self.activeCont, self.respCB) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.cGoal)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.cGoal)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.cGoal)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.cGoal)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.cGoal)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    #for pose in poseSeq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.poseSeq[self.cGoal]
        rospy.loginfo("Sending goal pose "+str(self.cGoal+1)+" to Action Server")
        rospy.loginfo(str(self.poseSeq[self.cGoal]))
        self.client.send_goal(goal, self.dControl, self.activeCont, self.respCB)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
