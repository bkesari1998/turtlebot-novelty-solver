#!/usr/bin/env python

import rospy
from coffee_bot_srvs.srv import Action

class Manager(object):
    
    def __init__(self):

        rospy.init_node("manager")

        rospy.wait_for_service("/action_executor")
        rospy.loginfo("action_executor service active")

        self.plan_file_paths = ["/home/mulip/catkin_ws/src/coffee-bot/pddls/problem_2_0_1.plan", "/home/mulip/catkin_ws/src/coffee-bot/pddls/problem_2_0_2.plan"] # maybe make param?

        plans = []
        for plan_file in self.plan_file_paths:
            plans.append(self.read_plan(plan_file))

        self.action_executor_client = rospy.ServiceProxy("action_executor", Action)

        # Go until goal state reached
        for i in range(20):
            for plan in plans:
                for action in plan:
                    res = self.action_executor_client(action)

                    if not res.success:
                        rospy.loginfo(action[0])
                        rospy.loginfo(res.message)
                        break
                rospy.loginfo("Success: %d" % i)

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