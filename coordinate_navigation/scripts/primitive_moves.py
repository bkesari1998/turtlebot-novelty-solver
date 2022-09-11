#!/usr/bin/env python

import rospy

from coffee_bot_srvs.srv import PrimitiveAction

from kobuki_msgs.msg import BumperEvent

from geometry_msgs.msg import Twist

import math


class PrimativeMoveAction(object):
    def __init__(self):
        """
        Initializes ROS node containing primitive move service.
        """

        # Initialize ROS node
        rospy.init_node("primitive_move_actions")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("primitive_move_actions node active")

        # Primative move actions are forward, backward, counter_clockwise, clockwise
        # Get values for move actions from ros param

        self.primative_actions = [
            "forward",
            "turn_cc",
            "turn_c"
        ]

        self.bumper_flag = False
        self.bumper_last_pressed = rospy.Time.now()

        # Command velocity publisher
        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # Initialize service
        self.primative_move_srv = rospy.Service(
            "/primitive_move_actions",
            PrimitiveAction,
            self.move_action_srv_handler,
        )
        rospy.loginfo("/primitive_move_actions service active")

        # Subscribe to bumper
        self.bumper_sub =  rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_handler)

        while not rospy.is_shutdown():
            rospy.spin()

    def move_action_srv_handler(self, req):
        """
        Handles call to primative move action service.
        param req: PrimativeAction request
        returns: PrimativeAction response.
        """

        if req.action in self.primative_actions:
            if req.action == "forward":
                status = self.forward()
            elif req.action == "turn_cc":
                status = self.turn_cc()
            else:
                status = self.turn_c()

            if status:
                return True, "Turtlebot successfully execututed primative move action"
            else:
                return False, "Turtlebot unable to execute primitive move action"
            
    def forward(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.linear.x = 0.25
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            else:
                self.bumper_flag = False
                return False
                

        return True

    def turn_cc(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.angular.z = math.pi / 6
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            else:
                self.bumper_flag = False
                return False

        return True

    def turn_c(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.angular.z = - math.pi / 6
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            else:
                self.bumper_flag = False
                return False

        return True


    def bumper_handler(self, msg):
        if msg.state == BumperEvent.PRESSED:
            if rospy.Time.now() - self.bumper_last_pressed < rospy.Duration(0.5):
                return
            self.bumper_last_pressed = rospy.Time.now()
            msg = Twist()
            msg.linear.x = -0.25
            for i in range(3):
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            self.cmd_vel.publish(Twist())
            self.rate.sleep()
            self.bumper_flag = True

    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("primitive_move_action node shutdown")

if __name__ == '__main__':
    PrimativeMoveAction()