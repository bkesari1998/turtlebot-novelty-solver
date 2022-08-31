#!/usr/bin/env python

import rospy

from coffee_bot_srvs.srv import PrimitiveAction

from kobuki_msgs.msg import BumperEvent

from geometry_msgs.msg import Twist


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

        # Command velocity publisher
        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)


        # try:
        #     param_primative_action_values = rospy.get_param(
        #         "/primative_move_actions"
        #     )
        #     if type(param_primative_action_values) != dict:
        #         raise TypeError

        #     if (
        #         set(param_primative_action_values.keys())
        #         != set(self.primative_action_values.keys())
        #     ):
        #         raise ValueError

        #     self.primative_action_values = param_primative_action_values
        # except rospy.ROSException:
        #     rospy.logwarn(
        #         "Parameter server reported an error. Resorting to in-node default primative action values"
        #     )
        # except KeyError:
        #     rospy.logwarn(
        #         "Value not set and default not given for '/primative_move_actions'. Resorting to in-node default primative action values"
        #     )
        # except TypeError:
        #     rospy.logwarn(
        #         "Value of param '/primative_move_actions' is not of type 'dict'. Resorting to in-node primative action values."
        #     )
        # except ValueError:
        #     rospy.logwarn(
        #         "Value of param '/primative_move_actions' contains unexpected keys. Resorting to in-node primative action values"
        #     )

        # Initialize service
        self.primative_move_srv = rospy.Service(
            "/primitive_move_actions",
            PrimitiveAction,
            self.move_action_srv_handler,
        )
        rospy.loginfo("/primitive_move_actions service active")

        # Subscribe to bumper
        self.bumper_sub =  rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_handler)

        # Initialize action client
        # self.action_client = actionlib.SimpleActionClient(
        #     "turtlebot_move", TurtlebotMoveAction
        # )
        # self.action_client.wait_for_server()

        while not rospy.is_shutdown():
            rospy.spin()

    def move_action_srv_handler(self, req):
        """
        Handles call to primative move action service.
        param req: PrimativeAction request
        returns: PrimativeAction response.
        """

        # # Create goal
        # action_goal = TurtlebotMoveGoal()

        if req.action in self.primative_actions:

            if req.action == "foward":
                status = self.forward()
            elif req.action == "turn_cc":
                status = self.turn_cc()
            else:
                status = self.turn_c()

            if status:
                return True, "Turtlebot successfully execututed primative move action"
            else:
                return False, "Turtlebot unable to execute primitive move action"
            

        # if req.action in self.primative_action_values.keys():
        #     if "turn" in req.action:
        #         action_goal.forward_distance = 0
        #         action_goal.turn_distance = self.primative_action_values[req.action]
        #     else:
        #         action_goal.turn_distance = 0
        #         action_goal.forward_distance = self.primative_action_values[req.action]
        # else:
        #     # Service request provided unexpected action
        #     return False, "Provided action is not included as a primative action"

        # # Send goal
        # if (
        #     self.action_client.send_goal_and_wait(
        #         action_goal, rospy.Duration(5), rospy.Duration(5)
        #     )
        #     == GoalStatus.SUCCEEDED
        # ):
        #     return True, "Turtlebot successfully execututed primative move action"
        # else:
        #     return False, "Turtlebot failed to execute primative move action"

    def forward(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.linear.x = 0.1
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            else:
                return False

        return True

    def turn_cc(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.angular.z = 0.1
                self.cmd_vel.publish(msg)
                self.rate.sleep()
            else:
                return False

        return True

    def turn_c(self):
        
        for i in range(10):
            if not self.bumper_flag:
                msg = Twist()
                msg.angular.z = -0.1
                self.cmd_vel.publish(msg)
                self.rate.sleep
            else:
                return False

        return True


    def bumper_handler(self, msg):
        if msg.state == BumperEvent.PRESSED:
            self.bumper_flag = True

    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("primitive_move_action node shutdown")

if __name__ == '__main__':
    PrimativeMoveAction()