#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from sound_play.msg import SoundRequest

from std_srvs.srv import Empty
from coffee_bot_srvs.srv import Open_Door

class OpenDoor(object):

    def __init__(self):

        # Initialize node
        rospy.init_node("open_door", anonymous=False)
        rospy.loginfo("open_door node active")
        rospy.on_shutdown(self.shutdown)

        # Initialize service
        self.open_door_srv = rospy.Service("/open_door", Open_Door, self.open_door)
        rospy.loginfo("open_door service active")

        # Clear costmaps service
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        # Robotsound publisher
        self.robot_sound_pub = rospy.Publisher("robotsound", SoundRequest, queue_size=1)
        self.rate = rospy.Rate(1)

        # Create sound message
        self.sound_msg = SoundRequest()
        self.sound_msg.sound = SoundRequest.SAY
        self.sound_msg.arg = "Please open the door."

        while not rospy.is_shutdown():
            rospy.spin()

    def open_door(self, req):
        """
        Service request handler.
        req: Trigger object.
        returns: Service response.
        """
        
        # Ask to open door, sleep 15 seconds
        self.robot_sound_pub.publish(self.sound_msg)
        rospy.sleep(15)

        # Clear costmaps
        self.clear_costmaps()

        return True, "Asked to open door"

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
