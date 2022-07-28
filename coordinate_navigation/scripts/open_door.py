#!/usr/bin/env python

import rospy
import os

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from coffee_bot_srvs.srv import Open_Door
from state.waypoints import state_check

class OpenDoor(object):

    def __init__(self):

        # Initialize node
        rospy.init_node("open_door", anonymous=False)
        rospy.loginfo("open_door node active")
        rospy.on_shutdown(self.shutdown)

        # Initialize service
        self.open_door_srv = rospy.Service("/open_door", Open_Door, self.open_door)
        rospy.loginfo("open_door service active")

        rospy.wait_for_service("/move_base/clear_costmaps")
        self.clear_costmap = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        self.door_open = False

        while not rospy.is_shutdown():
            rospy.spin()

    def set_door_open(self, msg, key):
        """
        Setter for door_open
        returns: none
        """
        if msg.data == -1:
            self.door_open = not state_check[key]['in_view']
        else:
            self.door_open = state_check[key]['in_view']

    def open_door(self, req):
        """
        Service request handler.
        req: Trigger object.
        returns: Service response.
        """
        door = req.door
        key = door + "_" + req.room + "_open"
        april_tag = state_check[key]["tag"]
        
        door_sub = rospy.Subscriber(april_tag, Float64, self.set_door_open, callback_args=key)
        rospy.loginfo(self.door_open)

        while not self.door_open:

            os.system("roslaunch coordinate_navigation open_door.launch")
            # Sleep for 5 seconds after asking to open door
            rospy.sleep(1)
            
            rospy.loginfo(self.door_open)
        
        self.clear_costmap()
    
        door_sub.unregister()
        return True, "Door is open"

    def shutdown(self):
        """
        Called on node shutdown.
        """

        rospy.loginfo("Stopping open_door node")

if __name__ == "__main__":
    try:
        OpenDoor()
    except:
        rospy.logerr("OpenDoor failed")
