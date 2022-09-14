#!/usr/bin/env python

import rospy
from coffee_bot_srvs.srv import Goal
import os


class PddlProblemGen(object):

    def __init__(self):
        """
        Initializes pddl_problem_gen node and service.
        returns: none
        """

        # Initialize ROS node
        rospy.init_node("pddl_problem_gen")
        rospy.loginfo("pddl_problem_gen node active")

        self.agents = rospy.get_param("/agents")
        self.object_list = rospy.get_param("/object_types")
        self.object_predicates = rospy.get_param("/objects")

        # Initialize service
        self.problem_gen_srv = rospy.Service("problem_gen", Goal, self.generate_problem)
        rospy.loginfo("problem_gen service active")

        while not rospy.is_shutdown():
            rospy.spin()

    def generate_problem(self, req):
        """
        Handles service call. Generates pddl problem file from world_state.py
        """

        # Load rosparams
        self.agents = rospy.get_param("/agents")
        self.object_list = rospy.get_param("/object_types")
        self.object_predicates = rospy.get_param("/objects")

        # Create/Open file
        path = "gen_pddls/problem_exploration.pddl"
        fp = open(path, "w")

        # Add problem definition to file
        problem_def = "(define (problem problem_exploration) (:domain coffee_bot)\n"
        fp.write(problem_def)
        # Parse dict
        for object_type, objects in self.object_list.items():
            object_line = ""
            for obj in objects:
                # Append object to line
                object_line += (obj + " ")

            # Append object type to line
            object_line += "- %s\n" % object_type
            fp.write("\t%s" % object_line)
        fp.write(")\n\n")

        fp.write("(:init\n")
        # Add initial state of agent
        for predicate_name, value in self.agents["turtlebot"].items():

            # Check if predicate is a boolean
            if type(value) == bool:
                if not value:
                    state_line = "(not(%s))\n" % predicate_name
                else:
                    state_line = "(%s)\n" % predicate_name
            # Predicate is not boolean
            else:
                state_line = "(%s %s)\n" % (predicate_name, value)
            fp.write("\t%s" % state_line)

        # Add initial state of other objects
        for object, predicate_dict in self.object_predicates.items():
            for predicate, value in predicate_dict.items():
                other_objects = ""
                # Check if predicate is boolean
                if type(value) == bool:
                    if not value:
                        state_line = "(not(%s %s))\n" % (predicate, object)
                    else:
                        state_line = "(%s %s)\n" % (predicate, object)
                # Predicate is not boolean
                else:
                    # Check if value is a list
                    if type(value) == list:
                        # Parse list
                        for val in value:
                            if value.index(val) == len(value) - 1:
                                other_objects += val
                            else:
                                other_objects += val + " "
                    # Value is not a list 
                    else:
                        other_objects = value

                    state_line = "(%s %s %s)\n" % (predicate, other_objects, obj)

                fp.write("\t%s" % state_line)

        fp.write(")\n\n")

        # Write goal to file
        fp.write("(:goal (and\n")
        for state in req.goal:
            fp.write("\t(%s)\n" % state)

        fp.write("))\n)")
        fp.close()

        return True
        
if __name__ == '__main__':
    PddlProblemGen()