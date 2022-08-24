#!/usr/bin/env python

import rospy
import numpy as np
from coffee_bot_srvs.srv import PrimitiveAction
import math

primative_action_values = {
    "forward": 0.1,
    "backward": -0.1,
    "turn_cc": math.pi / 3,
    "turn_c": - 3 * math.pi / 3,
}

actions = list(primative_action_values.keys())


if __name__ == '__main__':
    rospy.init_node("random_choice")
    rand_action_client = rospy.ServiceProxy("/primitive_move_actions", PrimitiveAction)
    
    counter = 0
    while not rospy.is_shutdown():
        action = np.random.choice(actions)
        rospy.loginfo(action)
        rand_action_client(action)
        counter += 1
        rospy.loginfo(counter)
        rospy.Rate(10).sleep()


