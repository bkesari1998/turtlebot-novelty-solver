#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger
from kobuki_msgs.msg import PowerSystemEvent

charge_status = False

def dock_tb(req):
    """
    Service request handler.
    req: Trigger object.
    """

    global charge_status

    if not charge_status:

        # Run shell command to launch auto docking node
        os.system("roslaunch kobuki_auto_docking activate.launch")
        rospy.sleep(1)

    if charge_status:
        return True, "Turtlebot Docked"
    
    return False, "Dock action failed"

def set_charge_status(msg):

    global charge_status
    
    if msg.event == msg.PLUGGED_TO_DOCKBASE:
        charge_status = True
    elif msg.event == msg.UNPLUGGED:
        charge_status = False

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

    # Create subscriber to check if tb is charging
    charge_sub = rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, set_charge_status)

    # Initialize service
    dock_srv = rospy.Service("/dock", Trigger, dock_tb)
    rospy.loginfo("dock service active")

    while not rospy.is_shutdown():
        rospy.spin()
