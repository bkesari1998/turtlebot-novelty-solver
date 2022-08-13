#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal

from std_srvs.srv import Trigger

def dock_tb(req):
    """
    Service request handler.
    req: Trigger object.
    """

    # Create simple action client 
    client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
    while not client.wait_for_server(rospy.Duration(5.0)):
        if rospy.is_shutdown(): return
        print('Action server is not connected yet. still waiting...')

    # Send goal to server 
    goal = AutoDockingGoal()
    client.send_goal(goal)
    rospy.on_shutdown(client.cancel_goal)
    client.wait_for_result()

    # Check success or failure
    if (client.get_state() == GoalStatus.SUCCEEDED):
        
        return True, "Turtlebot successfully docked."
    
    return False, "Turtlebot did not successfully dock."


def shutdown():
    """
    Called on node shutdown.
    """

    rospy.loginfo("Stopping auto_dock node")

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("auto_dock", anonymous=False)
    rospy.loginfo("auto_dock node active")
    rospy.on_shutdown(shutdown)

    # Initialize service
    dock_srv = rospy.Service("/dock", Trigger, dock_tb)
    rospy.loginfo("dock service active")

    while not rospy.is_shutdown():
        rospy.spin()
