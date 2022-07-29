#!/usr/bin/env python 

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float64
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
        rospy.loginfo('In move_tb')

        # Get waypoint info
        try:
            waypoint = rospy.get_param('waypoints/%s' % req.waypoint)
        except (rospy.ROSException, KeyError):
            return False, 'Waypoint does not exist' 

        try:
            state_confirmation = rospy.get_param('state_confirmation/at_%s' % req.waypoint)
        except (rospy.ROSException, KeyError):
            return False, 'Waypoint does not have entry in state_confirmation dictionary'

        # Assign the turtlebot's goal
        tb_goal = self.assign_goal(waypoint[0], waypoint[1])
        self.simple_action_client.send_goal(tb_goal)
        self.simple_action_client.wait_for_result()


        # Get distance to the tag
        try:
            tag_distance = rospy.wait_for_message(state_confirmation['tag'], Float64, rospy.Duration(2))
        except rospy.ROSException:
            return False, "Tag not in view"
        except KeyError:
            return False, "Tag is not set for waypoint"
        
        # Check if tag is close enough
        try: 
            if tag_distance.data <= state_confirmation['distance']:
                return True, "Turtlebot successfully navigated to goal position"
            else:
                return False, "Tag is too far away"
        except KeyError:
            return False, 'Distance is not set for tag'

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