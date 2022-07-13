#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger

def open_door(req):
    '''
    Service request handler.
    req: Trigger object.
    returns: Service response.
    '''
    os.system("roslaunch coordinate_navigation open_door.launch")

    # Sleep for 10 seconds after asking to open door
    rospy.sleep(10)

    return True, "Asked to open door"

def shutdown():
    '''
    Called on node shutdown.
    '''

    rospy.loginfo("Stopping open_door node")

if __name__ == "__main__":

    # Initialize node
    rospy.init_node("open_door", anonymous=False)
    rospy.loginfo("open_door node active")
    rospy.on_shutdown(shutdown)

    # Initialize service
    open_door_srv = rospy.Service("/open_door", Trigger, open_door)
    rospy.loginfo("open_door service active")

    while not rospy.is_shutdown():
        rospy.spin()