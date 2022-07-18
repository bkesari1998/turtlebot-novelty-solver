#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class MoveToStart():
    def __init__(self):
        '''
        Initializes move_to_start ROS node.
        '''
        
        # Initialize node
        rospy.init_node("undock", anonymous=False)
        rospy.loginfo("undock node active")
        rospy.on_shutdown(self.shutdown)

        # Velocity command publisher
        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10) 
        self.rate = rospy.Rate(10)

        # Initialize service
        self.move_to_start_srv = rospy.Service("undock", Trigger, self.move_to_start)
        rospy.loginfo("undock service active")

        while not rospy.is_shutdown():
            rospy.spin()

    def move_to_start(self, req):
        '''
        Service handler. Calls functions to reverse and rotate the tb.
        req: Trigger object
        returns: Service response
        '''

        # reverse and rotate the turtlebot
        self.reverse()
        # self.rotate()

        return True, "Turtlebot successfully moved to starting position"

    def reverse(self):
        '''
        Publishes Twist messages to revese the turtlebot.
        '''

        # create msg to reverse turtlebot
        move_cmd = Twist()
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0

        for i in range(25):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def rotate(self):
        '''
        Publishes Twist messages to rotate the turtlebot.
        '''

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
        '''
        Called on node shutdown.
        '''
 
        rospy.loginfo("Stopping move_to_start node")
        rospy.loginfo("Stopping turtlebot")

        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        MoveToStart()
    except:
        rospy.logerr("MoveToStart failed")

