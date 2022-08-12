#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from kobuki_msgs.msg import PowerSystemEvent

class Undock():
    def __init__(self):
        """
        Initializes move_to_start ROS node.
        returns: none
        """
        
        # Initialize node
        rospy.init_node("undock", anonymous=False)
        rospy.loginfo("undock node active")
        rospy.on_shutdown(self.shutdown)

        # Velocity command publisher
        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10) 
        self.rate = rospy.Rate(10)

        # Initialize service
        self.move_to_start_srv = rospy.Service("undock", Trigger, self.undock)
        rospy.loginfo("undock service active")
        
        self.power_systems_sub = rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, self.set_charge_status)

        self.charge_status = True

        while not rospy.is_shutdown():
            rospy.spin()

    def set_charge_status(self, msg):

        if msg.event == msg.PLUGGED_TO_DOCKBASE:
            self.charge_status = True
        elif msg.event == msg.UNPLUGGED:
            self.charge_status = False

    def undock(self, req):
        """
        Service handler. Calls functions to reverse and rotate the tb.
        req: Trigger object
        returns: Service response
        """

        if self.charge_status:
            # reverse  the turtlebot
            self.reverse()

        if not self.charge_status:
            return True, "Turtlebot successfully undocked"

        return False, "Turtlebot failed to undock"

    def reverse(self):
        """
        Publishes Twist messages to reverse the turtlebot.
        returns: none
        """

        # create msg to reverse turtlebot
        move_cmd = Twist()
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0

        for i in range(25):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def rotate(self):
        """
        Publishes Twist messages to rotate the turtlebot.
        returns: none
        """

        # create msg to rotate turtlebot
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 3.1

        # publish msg
        for i in range(13):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

        # stop the turtlebot
        self.cmd_vel.publish(Twist())
        self.rate.sleep()

    def shutdown(self):
        """
        Called on node shutdown.
        returns: none
        """
 
        rospy.loginfo("Stopping move_to_start node")


if __name__ == "__main__":
    try:
        Undock()
    except:
        rospy.logerr("Undock failed")

