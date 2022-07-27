#!/usr/bin/env python

import rospy
import world_state
from std_srvs.srv import Trigger


class PddlProblemGen(object):

    def __init__(self):
        """
        Initializes pddl_problem_gen node and service.
        returns: none
        """

        # Initialize ROS node
        rospy.init_node("pddl_problem_gen")
        rospy.loginfo("pddl_problem_gen node active")
        rospy.on_shutdown(self.shutdown)

        # Initialize service
        self.problem_gen_srv = rospy.Service("problem_gen", Trigger, self.generate_problem)
        rospy.loginfo("problem_gen service active")

        # Counter for naming different problem files
        self.prob_count = 0

        while not rospy.is_shutdown():
            rospy.spin()

    def generate_problem(self):
        """
        Handles service call. Generates pddl problem file from world_state.py
        """

        # Create/Open file
        path = "problem_%d.pddl" % self.prob_count
        fp = open(path, "w")

        # Add problem definition to file
        problem_def = "(define (problem problem_%d) (:domain coffee_bot)\n" % self.prob_count
        fp.write(problem_def)

        # Add objects to file
        fp.write("(:objects\n")
        # Parse dict
        for object_type in world_state.objects:
            object_line = ""
            for obj in world_state.objects[object_type]:
                # Append object to line
                object_line += (obj + " ")

            # Append object type to line
            object_line += "- %s\n" % object_type
            fp.write("\t%s" % object_line)
        fp.write(")\n\n")

        fp.write("(:init\n")
        # Add initial state of agent
        for predicate, value in world_state.agents["turtlebot"].items():

            # Check if predicate is a boolean
            if type(value) == bool:
                if not value:
                    state_line = "(not(%s))\n" % predicate
                else:
                    state_line = "(%s)\n" % predicate
            # Predicate is not boolean
            else:
                state_line = "(%s %s)\n" % (predicate, value)
            fp.write("\t%s" % state_line)

        # Add initial state of other objects
        for object_type in world_state.objects:
            # Parse dict
            for obj in world_state.objects[object_type]:
                # Parse dict
                for predicate, value in world_state.objects[object_type][obj].items():
                    other_objects = ""
                    # Check if predicate is boolean
                    if type(value) == bool:
                        if not value:
                            state_line = "(not(%s %s))\n" % (predicate, obj)
                        else:
                            state_line = "(%s %s)\n" % (predicate, obj)
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
        fp.write("(:goal (and\n\t(facing desk_refill)\n))\n)")
        fp.close()
        self.prob_count += 1


if __name__ == '__main__':
    try:
        pddl = PddlProblemGen()
    except:
        rospy.loginfo("PddlProblemGen failed")
