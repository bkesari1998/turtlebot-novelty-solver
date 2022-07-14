# test
#!/usr/bin/env python
from waypoints_dict import waypoints
from world_state import *

import rospy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from coffee_bot_srvs.srv import Move, Plan

class PlanExecutor():

    global agents
    global doors
    global rooms
    global desks
    
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
                self.approach_door(action)
            # elif action[0] == 'open_door':
                # pass
            elif action[0] == 'exit_room':
                self.exit_room(action)
            # elif action[0] == 'approach_desk':
                # pass
            # elif action[0] == 'make_coffee':
                # pass
            elif action[0] == 'approach_desk_refill':
                self.approach_desk_refill(action)
            # elif action[0] == 'refill':
                # pass
            elif action[0] == 'approach_charger':
                pass
            elif action[0] == 'dock':
                self.dock(action)
            elif action[0] == 'undock':
                self.undock(action)
            elif action[0] == 'charge':
                pass

    def approach_door(self, action):
    
        door = action[1]
        room1 = action[2]
        room2 = action[3]

        # Precondition checking
        if (doors.has_key(door) and 
        room1 in door["connect"] and room2 in door["connect"] and 
        agents["turtlebot"]["at"] == room1 and 
        not self.turtlebot["docked"]):

            # Call move action
            status = self.move_action(door + "_" + room1)

            # Update world state
            if status:
                agents["turtlebot"]["facing"] = door

            # Return status
            return status

        return False

    def exit_room(self, action):

        room1 = action[1]
        room2 = action[2]
        door = action[3]

        # Precondition checking
        if doors.has_key(door):
            if ( room1 in doors[door]["connect"] and room2 in doors[door]["connect"] and
            agents["turtlebot"]["at"] == room1 and
            doors[door]["open"]):

                # Call move action
                status = self.move_action(door + "_" + room2)

                # Update world state
                if status:
                    agents["turtlebot"]["at"] == room2
                
                return status

        return False

    def approach_desk_refill(self, action):

        room1 = action[1]
        desk1 = action[2]

        # Precondition checking
        if (rooms.has_key(room1) and 
        desks.has_key(desk1)):
            if (desks[desk1]["in"] == room1 and 
            agents["turtlebot"]["at"] == room1 and
            not agents["turtlebot"]["docked"]):

                # Call move action
                status = self.move_action(desk1)

                # Update world state
                if status:
                    agents["turtlebot"]["facing"] == desk1
                
                return status

        return False

    def dock(self, action):

        room1 = action[1]
        charger1 = action[2]

        # Precondition checking
        if (rooms.has_key(room1) and 
        chargers.has_key(charger1)):
            if (agents["turtlebot"]["facing"] == "charger_1" and
            agents["turtlebot"]["at"] == room1 and 
            chargers[charger1]["inside"] == room1):

                status = self.dock_action()

                if status:
                    agents["turtlebot"]["docked"] == True
                    agents["turtlebot"]["facing"] == charger1

                return status

        return False

    def undock(self, action):

        room1 = action[1]
        charger1 = action[2]

        if (rooms.has_key(room1) and 
        chargers.has_key(charger1)):
            if (agents["turtlebot"]["facing"] == "charger_1" and
            agents["turtlebot"]["at"] == room1 and 
            chargers[charger1]["inside"] == room1):

                status = self.dock_action()

                if status:
                    agents["turtlebot"]["docked"] == False
                    agents["turtlebot"]["facing"] == charger1

                return status
    
        return False
    
    def approach_charger(self, action):
        
        room1 = action[1]
        charger1 = action[2]

        if (rooms.has_key(room1) and 
        chargers.has_key(charger1)):
            if (agents["turtlebot"]["at"] == room1 and 
            chargers[charger1]["inside"] == room1):

                status = self.move_action("dock_approach")

                if status:
                    agents["turtlebot"]["facing"] == charger1

                return status
                
            return False

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

        return response.success

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

            return response.success

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