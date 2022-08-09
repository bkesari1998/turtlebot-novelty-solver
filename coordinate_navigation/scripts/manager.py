#!/usr/bin/env python

import rospy
from coffee_bot_srvs.srv import Action

class Manager(object):
    
    def __init__(self):

        rospy.init_node("manager")

        rospy.wait_for_service("/action_executor")
        rospy.loginfo("action_executor service active")

        self.action_executor_client = rospy.ServiceProxy("action_executor", Action)

        self.plan_count = 0