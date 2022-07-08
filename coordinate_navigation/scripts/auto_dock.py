#!/usr/bin/env python

import rospy
import roslaunch

from std_msgs.msg import Bool

def dock(msg):
    node = roslaunch.core.Node("kobuki_auto_docking", "DockDriveActionClient.py")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(node)

if __name__ == "__main__":

    rospy.init_node("auto_dock", anonymous=False)

    sub = rospy.Subscriber("/dock", Bool, callback=dock)