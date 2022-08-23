#!/usr/bin/env python

import rospy
import numpy as np
from coffee_bot_srvs.srv import PrimitiveAction
import math

primative_action_values = {
    "forward": 0.1,
    "backward": -0.1,
    "turn_cc_60": math.pi / 3,
    "turn_cc_120": 2 * math.pi / 3,
    "turn_cc_180": math.pi,
    "turn_cc_240": 4 * math.pi / 3,
    "turn_cc_300": 5 * math.pi / 3,
    "turn_cc_360": 2 * math.pi,
    "turn_c_60": -math.pi / 3,
    "turn_c_120": -2 * math.pi / 3,
    "turn_c_180": -math.pi,
    "turn_c_240": -4 * math.pi / 3,
    "turn_c_300": -5 * math.pi / 3,
    "turn_c_360": -2 * math.pi,
}
actions = list(primative_action_values.keys())


if __name__ == '__main__':
    rospy.init_node("random_choice")
    rand_action_client = rospy.ServiceProxy("/primitive_move_actions", PrimitiveAction)
    
    counter = 0
    while not rospy.is_shutdown():
        rand_action_client(np.random.choice(primative_action_values))
        counter += 1
        rospy.loginfo(counter)
        rospy.Rate(10).sleep()


