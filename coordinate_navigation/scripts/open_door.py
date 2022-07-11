#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger

def open_door(req):
    os.system("roslaunch coordinate_navigation open_door.launch")

    # Sleep for 10 seconds after asking to open door
    rospy.sleep(10)

    
    return True, "Door open"

if __name__ == "__main__":

    rospy.init_node("open_door", anonymous=False)

    open_door_srv = rospy.Service("/open_door", Trigger, open_door)

    while not rospy.is_shutdown():
        rospy.spin()