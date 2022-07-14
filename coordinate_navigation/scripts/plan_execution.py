#!/usr/bin/env python
from pydoc import _OldStyleClass
from waypoints_dict import waypoints

import rospy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
from std_srvs.srv import Trigger
from coffee_bot_srvs.srv import Move, Plan

class PlanExecutor():
    
    def __init__(self):

        # Initialize node
        rospy.init_node("plan_execution", anonymous=False)
        rospy.loginfo("plan_execution node active")

        # Initialize service
        self.plan_executor_srv = rospy.Service("/plan_executor", Plan, self.execute_plan) 
        rospy.loginfo("plan_executor service active")

        # Wait for action services
        rospy.loginfo("Waiting for move_to_start service")
        rospy.wait_for_service("move_to_start")
        rospy.loginfo("Waiting for dock service")
        rospy.wait_for_service("dock")
        rospy.loginfo("Waiting for move service")
        rospy.wait_for_service("move")
        rospy.loginfo("Waiting for open_door service")
        rospy.wait_for_service("/open_door")
        rospy.loginfo("Waiting for amcl")
        rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10)

        rospy.loginfo("All services running")

        while not rospy.is_shutdown():
            rospy.spin()
        
    def execute_plan(self, req):
        
        # Loop over actions 
        for action in req.plan:

            # Split action by space 
            action = action.split()

            if action[0] == 'approach_door':
                if action[2] == 'lab':
                    self.move_action('lab_door_in')
                else:
                    self.move_action('lab_door_out')

            elif action[0] == 'open_door':
                pass
            elif action[0] == 'exit_room':
                pass
            elif action[0] == 'approach_desk':
                pass
            elif action[0] == 'make_coffee':
                pass
            elif action[0] == 'approach_desk_refill':
                pass
            elif action[0] == 'refill':
                pass
            elif action[0] == 'approach_charger':
                pass
            elif action[0] == 'dock':
                pass
            elif action[0] == 'undock':
                pass
            elif action[0] == 'charge':
                pass

            

    def start_action(self):

        # Call to service
        try:
            move_to_start = rospy.ServiceProxy("move_to_start", Trigger)
            response = move_to_start()
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)
        
    def dock_action(self):

        # Call to service
        try:
            dock = rospy.ServiceProxy("dock", Trigger)
            response = dock()
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def open_door_action(self):

        # Call to service
        try:
            open_door = rospy.ServiceProxy("open_door", Trigger)
            response = open_door()
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def move_action(self, loc):
        # Call to service
        try:
            move_tb = rospy.ServiceProxy("move", Move)
            response = move_tb(waypoints[loc][0], waypoints[loc][1])
            rospy.loginfo(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def shutdown(self):

        rospy.loginfo("Stopping plan_executor node")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    
    try:
        PlanExecutor()
    except:
        rospy.logerr("PlanExecutor failed")