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

        self.door_open = False

        while not rospy.is_shutdown():
            rospy.spin()

    def set_door_open(self, msg, in_view):
        """
        Setter for door_open
        returns: none
        """

        if msg.data == -1:
            self.door_open = not in_view
        else:
            self.door_open = in_view

    def open_door(self, req):
        """
        Service request handler.
        req: Trigger object.
        returns: Service response.
        """

        try:
            state_confirmation = rospy.get_param('state_confirmation/%s_%s_open' % (req.door, req.room))
        except (rospy.ROSException, KeyError):
            return False, 'Door does not have entry in state_confirmation dictionary'

        try:
            april_tag = state_confirmation['tag']
        except KeyError:
            return False, 'Tag not set for door'

        try:
            in_view = state_confirmation['in_view']
        except KeyError:
            return False, 'In_view not set for door'    
        
        door_sub = rospy.Subscriber(april_tag, Float64, self.set_door_open, callback_args=in_view)

        while not self.door_open:

            os.system("roslaunch coordinate_navigation open_door.launch")
            # Sleep for 5 seconds after asking to open door
            rospy.sleep(1)
    
        door_sub.unregister()

        # Set door flag back to false for next use
        self.door_open = False

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
