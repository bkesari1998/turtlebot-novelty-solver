#!/usr/bin/env python 

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from coffee_bot_srvs.srv import Move
from geometry_msgs.msg import Twist

class MoveTB():
    def __init__(self):
        '''
        Initializes move_tb ROS node.
        '''

        # Initialize node
        rospy.init_node("move_tb", anonymous=False)
        rospy.loginfo("move_tb node active")
        rospy.on_shutdown(self.shutdown)

        # Velocity command publisher for shutdown
        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        # Initialize service
        self.move_tb_srv = rospy.Service("/move", Move, self.move_tb)
        rospy.loginfo("move service active")

        # Create a SimpleActionClient of a move_base action type and wait for server
        self.simple_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.simple_action_client.wait_for_server()

        while not rospy.is_shutdown():
            rospy.spin()

    def assign_goal(self, pose, orientation):
        """
        Assigns a pose and orientation to a MoveBaseGoal object.
        pose: Cartiesian pose list [x, y, z]
        orientation: Quaternion rotation list [x, y ,z ,w]
        returns: MoveBaseGoal object
        """

        # Create MoveBaseGoal object from input
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0]
        goal_pose.target_pose.pose.position.y = pose[1]
        goal_pose.target_pose.pose.position.z = pose[2]
        goal_pose.target_pose.pose.orientation.x = orientation[0]
        goal_pose.target_pose.pose.orientation.y = orientation[1]
        goal_pose.target_pose.pose.orientation.z = orientation[2]
        goal_pose.target_pose.pose.orientation.w = orientation[3]

        return goal_pose

    def move_tb(self, req):
        """
        Handler for move_tb service. Calls SimpleAction service to move tb to goal position.
        req: Service request, a Move object.
        returns: Service response.
        """

        # Assign the turtlebot's goal
        tb_goal = self.assign_goal(req.final_pose, req.final_orientation)
        self.simple_action_client.send_goal(tb_goal)
        self.simple_action_client.wait_for_result()

        if (self.simple_action_client.get_state() == GoalStatus.SUCCEEDED or 
        self.simple_action_client.get_state() == GoalStatus.PREEMPTED or 
        self.simple_action_client.get_state() == GoalStatus.PREEMPTING):
            return True, "Turtlebot successfully navigated to goal position"
        else:
            return False, "Turtlebot unable to navigate to goal position"

    def shutdown(self):
        """
        Called on shutdown of node.
        """
        
        rospy.loginfo("Stopping move_tb node")
        rospy.loginfo("Stopping turtlebot")
        
        # Stop the turtlebot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        MoveTB()
    except:
        rospy.logerr("MoveTB failed")