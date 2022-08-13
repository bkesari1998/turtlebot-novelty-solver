#!/usr/bin/env python

import rospy
from coffee_bot_srvs.srv import Action

class Manager(object):
    
    def __init__(self):

        rospy.init_node("manager")

        rospy.wait_for_service("/action_executor")
        rospy.loginfo("action_executor service active")

        self.plan_file_path = "/home/mulip/catkin_ws/src/coffee-bot/pddls/problem_2_0_1.plan" # maybe make param?
        self.goal_state = "facing desk_1"

        plan = self.read_plan(self.plan_file_path)

        self.action_executor_client = rospy.ServiceProxy("action_executor", Action)

        # Go until goal state reached
        
        for action in plan:
            res = self.action_executor_client(action)

            if not res.success:
                rospy.loginfo(action[0])
                rospy.loginfo(res.message)
                break

    def read_plan(self, plan_file_path):
        rospy.loginfo("In read plan")
        f = open(plan_file_path, "r")

        # Get lines of file
        lines = f.readlines()


        # Only get plan lines
        plan_lines = lines[3: -3]

        # Remove timestamp from plan lines
        for i in range(len(plan_lines)):

            plan_lines[i] = plan_lines[i][plan_lines[i].index("(") + 1: plan_lines[i].index(")")]

        # Return plan, with each action split by space char
        plan = []
        for line in plan_lines:
            plan.append(line.split(" "))

        f.close()

        return plan

if __name__ == "__main__":
    Manager()