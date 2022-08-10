#!/usr/bin/python

# Utility
import rospy

# ROS Messages/Services
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Trigger, Empty
from coffee_bot_srvs.srv import Move, Open_Door, Action

class PlanExecutor():

    def __init__(self):

        '''
        Initializes action_execution node.
        '''

        # Initialize node
        rospy.init_node("action_execution", anonymous=False)
        rospy.loginfo("action_execution node active")
        rospy.on_shutdown(self.shutdown)

        # Initialize velocit publisher
        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10) 
        self.rate = rospy.Rate(10)

        # Get state parameters
        self.agents = rospy.get_param("agents")
        self.objects = rospy.get_param("objects")

        # Wait for action services
        rospy.loginfo("Waiting for undock service")
        rospy.wait_for_service("undock")
        rospy.loginfo("Waiting for dock service")
        rospy.wait_for_service("dock")
        rospy.loginfo("Waiting for move service")
        rospy.wait_for_service("move")
        rospy.loginfo("Waiting for amcl")
        rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10)
        rospy.loginfo("Waiting for open_door service")
        rospy.wait_for_service("open_door")
        rospy.loginfo("Waiting for primitive_move_actions service")
        rospy.wait_for_service("primitive_move_actions")
        rospy.loginfo("Waiting for confirm_state service")
        rospy.wait_for_service("confirm_state")
        rospy.loginfo("All services running")

        # Initialize service
        self.action_executor_srv = rospy.Service("/action_executor", Action, self.execute_action) 
        rospy.loginfo("action_executor service active")
    
        # Create primitive move service proxy
        self.prim_move_client = rospy.ServiceProxy("/primitive_move_actions", Action)

        # Create state_confirmer service proxy
        self.state_conf_client = rospy.ServiceProxy("/confirm_state", Empty)


        while not rospy.is_shutdown():
            rospy.spin()
        
    def execute_action(self, req):

        '''
        Executes action, either symbolic or primitive.
        req: Action() object containing list of strings
        returns: Status of action (bool, string)
        '''

        if req.action[0] == 'approach':
           return self.approach(req.action)

        if req.action[0] == 'open_door':
            return self.open_door(req.action)
        
        if req.action[0] == 'pass_through_door':
            return self.pass_through_door(req.action)
        
        if req.action[0] == 'dock':
            return self.dock(req.action)
        
        if req.action[0] == 'undock':
            return self.undock(req.action)
        
        if req.action[0] == 'move':
            return self.primitive_move(req.action[1])

        return False, "Unknown action provided"

    

##################### Symbolic Actions #####################

    def approach(self, action):
        '''
        approach action executor, checks pre and post conditions of action.
        Calls move_action to move the turtlebot.
        action: list of strings expressing the pddl approach door action
        returns: boolean representing success/failure of action
        '''

        object_1 = action[1]
        object_2 = action[2]
        room_1 = action[3]

        # Precondition checking
        try:
            if room_1 in self.objects[object_1]["inside"] and room_1 in self.objects[object_2]["inside"] \
                and room_1 in self.agents["turtlebot"]["at"] and object_1 in self.agents["turtlebot"]["facing"]:
                
                # Get waypoint
                waypoint = ""
                # If door, append room info
                if "door" in object_1:
                    waypoint = object_2 + "_" + room_1 + "_" + "_" + room_1
                else:
                    waypoint = object_1


                # Call to move service
                self.move_action(waypoint)

                # State update
                self.update_state()
                facing = self.agents["turtlebot"]["facing"]

                # Postcondition checking
                if object_2 in facing and object_1 not in facing:

                    return True, "Action succeeded."
                
                return False, "Postconditions not met."

            return False, "Preconditions not met." 


        except KeyError:
            return False, "Key error when looking up state."

    def open_door(self, action):

        '''
        open_door action executor. Checks pre and post conditions of action
        Call open_door_action.
        action: list of strings expressing the pddl open door action
        returns: boolean representing success/failure of action
        '''
    
        door_1 = action[1]
        wall_1 = action[2]

        try:
            if door_1 in self.agents["turtlebot"]["facing"] and \
            wall_1 not in self.agents["turtlebot"]["facing"] and \
            self.objects[door_1]["open"] == False:
                
                # Call to open_door service
                self.open_door_action(door_1)

                # State update
                self.update_state()
                facing = self.agents["turtlebot"]["facing"]
                door_open = self.objects[door_1]["open"]
                
                # Postcondition checking
                if door_open == True and \
                door_1 not in facing and wall_1 in facing:
                    return True, "Action succeeded."
                
                return False, "Postconditions not met."
            
            return False, "Preconditions not met."

        except KeyError:
            return False, "Key error when looking up state."

    def pass_through_door(self, action):

        '''
        go_through_door action executor, checks pre and post conditions of action.
        Calls move_action to move the turtlebot.
        action: list of strings expressing the pddl approach door action
        returns: boolean representing success/failure of action
        '''

        room_1 = action[1]
        room_2 = action[2]
        door_1 = action[3]

        try:
            if room_1 in self.agents["turtlebot"]["at"] and \
            room_1 in self.objects[door_1]["connect"] and \
            room_2 in self.objects[door_1]["connect"] and \
            self.objects[door_1]["open"] == True:
                
                waypoint = door_1 + "_" + room_2 + "_" + room_1

                # Call to open_door service
                self.move_action(waypoint)

                # State update
                self.update_state()
                at = self.agents["turtlebot"]["at"]
                
                # Postcondition checking
                if room_2 in at and room_1 not in at:
                    return True, "Action succeeded."
                
                return False, "Postconditions not met."
            
            return False, "Preconditions not met."

        except KeyError:
            return False, "Key error when looking up state."


    def dock(self, action):

        '''
        dock action executor, checks pre and post conditions of action.
        Calls dock_action to dock the turtlebot.
        action: list of strings expressing the pddl approach door action
        returns: boolean representing success/failure of action
        '''

        room_1 = action[1]
        charger_1 = action[2]
  
        try:
            if room_1 in self.agents["turtlebot"]["at"] and \
            charger_1 in self.agents["turtlebot"]["facing"] and \
            not self.agents["turtlebot"]["docked"] and \
            charger_1 in self.objects[room_1]:

                # Call to dock service
                self.dock_action()

                # State update
                self.update_state()
                docked = self.agents["turtlebot"]["docked"]
                
                # Postcondition checking
                if docked:
                    return True, "Action succeeded."
                
                return False, "Postconditions not met."
            
            return False, "Preconditions not met."

        except KeyError:
            return False, "Key error when looking up state."

    def undock(self, action):

        '''
        undock action executor, checks pre and post conditions of action.
        Calls undock_action to move the turtlebot.
        action: list of strings expressing the pddl approach door action
        returns: boolean representing success/failure of action
        '''

        room_1 = action[1]
        charger_1 = action[2]
  
        try:
            if room_1 in self.agents["turtlebot"]["at"] and \
            self.agents["turtlebot"]["docked"] and \
            charger_1 in self.objects[room_1]:

                # Call to dock service
                self.undock_action()

                # State update
                self.update_state()
                docked = self.agents["turtlebot"]["docked"]
                at = self.agents["turtlebot"]["at"]
                facing = self.agents["turtlebot"]["facing"]
                
                # Postcondition checking
                if not docked and facing == charger_1 and at == room_1:
                    return True, "Action succeeded."
                
                return False, "Postconditions not met."
            
            return False, "Preconditions not met."

        except KeyError:
            return False, "Key error when looking up state."
    

##################### Service Callers #####################

    def undock_action(self):

        '''
        Service call to undock.
        returns: none
        '''

        # Call to service
        try:
            move_to_start = rospy.ServiceProxy("undock", Trigger)
            response = move_to_start()
            rospy.loginfo(response.message)
            if response.success:
                return True
        except rospy.ServiceException as e:
            rospy.logerr(e)
        
        return False
        
    def dock_action(self):

        '''
        Service call to dock.
        returns: none
        '''

        # Call to service
        try:
            dock = rospy.ServiceProxy("dock", Trigger)
            response = dock()
            rospy.loginfo(response.message)
            if response.success:
                return True
        except rospy.ServiceException as e:
            rospy.logerr(e)

        return False

    def open_door_action(self, door, room):

        '''
        Service call to open_door.
        returns: none.
        '''

        # Call to service
        try:
            open_door = rospy.ServiceProxy("open_door", Open_Door)

            response = open_door(door, room)
            rospy.loginfo(response.message)
            if response.success:
                return True
        except rospy.ServiceException as e:
            rospy.logerr(e)
        
        return False

    def move_action(self, loc):

        '''
        Service call to move_action.
        returns: none.
        '''

        # Call to service
        try:
            move_tb = rospy.ServiceProxy("move", Move)
            response = move_tb(loc)
            rospy.loginfo(response.message)
            if response.success:
                return True
        except rospy.ServiceException as e:
            rospy.logerr(e)

        return False

##################### Primative Move Actions #####################

    def primitive_move(self, action):
        """
        Calls primitive move service.
        action: String representing primitive move.
        returns: Result of service call (bool, string)
        """

        # Make call to primitive move service
        result = self.prim_move_client(action)

        return result

##################### State Update ####################

    def update_state(self):
        
        self.state_conf_client()
        self.agents = rospy.get_param("agents")
        self.objects = rospy.get_param("objects")

##################### Shutdown #####################

    def shutdown(self):

        '''
        Runs on node shutdown.
        returns: none
        '''

        rospy.loginfo("Stopping plan_executor node")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)



if __name__ == "__main__":
    
    try:
        PlanExecutor()
    except:
        rospy.logerr("PlanExecutor failed")