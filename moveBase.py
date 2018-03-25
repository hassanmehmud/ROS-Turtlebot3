#!/usr/bin/env python
#Move turtlebot3 from location A to location B through move_base, action client
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
#define goal/pose
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.5
	goal.target_pose.pose.orientation.w = 1.0
# send goal and wait for result (Todo: shrink delay)
	client.send_goal(goal)
	wait = client.wait_for_result()
# handle if navigation stack is not running for move_base then shutdown otherwise retrieve results
	if not wait:
	    rospy.logerr("Server not available!")
	    rospy.signal_shutdown("Server not available!")
	else:
	    return client.get_result()
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Reached goal!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt, navigation finnished.")

