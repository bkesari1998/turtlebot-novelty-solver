#!/usr/bin/env python

import rospy

from coffee_bot_srvs.srv import Action

from actionlib_msgs.msg import GoalStatus
import actionlib
from turtlebot_actions.msg import TurtlebotMoveAction, TurtlebotMoveGoal


class PrimativeMoveAction(object):
    def __init__(self):
        """
        Initializes ROS node containing primitive move service.
        """

        # Initialize ROS node
        rospy.init_node("primative_move_actions")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("primitive_move_actions node active")

        # Primative move actions are forward, backward, counter_clockwise, clockwise
        # Get values for move actions from ros param

        self.primative_action_values = {
            "forward": 0.1,
            "backward": -0.1,
            "counter_clockwise": 0.5,
            "clockwise": -0.5,
        }

        try:
            param_primative_action_values = rospy.get_param(
                "/primative_move_actions"
            )
            if type(param_primative_action_values) != dict:
                raise TypeError

            if (
                set(param_primative_action_values.keys())
                != set(self.primative_action_values.keys())
            ):
                raise ValueError

            self.primative_action_values = param_primative_action_values
        except rospy.ROSException:
            rospy.logwarn(
                "Parameter server reported an error. Resorting to in-node default primative action values"
            )
        except KeyError:
            rospy.logwarn(
                "Value not set and default not given for '/primative_move_actions'. Resorting to in-node default primative action values"
            )
        except TypeError:
            rospy.logwarn(
                "Value of param '/primative_move_actions' is not of type 'dict'. Resorting to in-node primative action values."
            )
        except ValueError:
            rospy.logwarn(
                "Value of param '/primative_move_actions' contains unexpected keys. Resorting to in-node primative action values"
            )

        # Initialize service
        self.primative_move_srv = rospy.Service(
            "/primative_move_actions",
            Action,
            self.move_action_srv_handler,
        )
        rospy.loginfo("/primative_move_actions service active")

        # Initialize action client
        self.action_client = actionlib.SimpleActionClient(
            "turtlebot_move", TurtlebotMoveAction
        )
        self.action_client.wait_for_server()

        while not rospy.is_shutdown():
            rospy.spin()

    def move_action_srv_handler(self, req):
        """
        Handles call to primative move action service.
        param req: PrimativeAction request
        returns: PrimativeAction response.
        """

        # Create goal
        action_goal = TurtlebotMoveGoal()

        if req.action == "forward":
            action_goal.forward_distance = self.primative_action_values["forward"]
            action_goal.turn_distance = 0
        elif req.action == "backward":
            action_goal.forward_distance = self.primative_action_values["backward"]
            action_goal.turn_distance = 0
        elif req.action == "counter_clockwise":
            action_goal.forward_distance = 0
            action_goal.turn_distance = self.primative_action_values[
                "counter_clockwise"
            ]
        elif req.action == "clockwise":
            action_goal.forward_distance = 0
            action_goal.turn_distance = self.primative_action_values["clockwise"]
        else:
            # Service request provided unexpected action
            return False, "Provided action is not included as a primative action"

        # Send goal
        if (
            self.action_client.send_goal_and_wait(
                action_goal, rospy.Duration(2), rospy.Duration(2)
            )
            == GoalStatus.SUCCEEDED
        ):
            return True, "Turtlebot successfully execututed primative move action"
        else:
            return False, "Turtlebot failed to execute primative move action"


    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("primitive_move_action node shutdown")

if __name__ == '__main__':
    PrimativeMoveAction()