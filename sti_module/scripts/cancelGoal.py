#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
   
    client.cancel_all_goals()
    rospy.loginfo("cancel_all_goals")


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_cancel_py')
        movebase_client()
      
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")