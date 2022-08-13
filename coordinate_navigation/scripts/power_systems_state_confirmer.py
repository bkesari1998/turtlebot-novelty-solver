#!/usr/bin/python

import rospy
from kobuki_msgs.msg import PowerSystemEvent

def handler(msg):
    rospy.loginfo(msg.event)
    if msg.event == PowerSystemEvent.PLUGGED_TO_DOCKBASE:
        rospy.set_param("agents/turtlebot/docked", True)
    elif msg.event == PowerSystemEvent.UNPLUGGED:
        rospy.set_param("agents/turtlebot/docked", False)


if __name__ == "__main__":

    rospy.init_node("power_systems_event_sub")
    
    pse_sub = rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, handler)

    while not rospy.is_shutdown():
        rospy.spin()