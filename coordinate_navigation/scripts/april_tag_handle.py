#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray


class AprilTagHandler(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tag_handler")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("tag_handler node active")

        # Initialize tag_detections publishers
        self.tag_ids = [0, 1] # Make this a rosparam later
        self.tag_pubs = []
        for id_num in self.tag_ids:
            pub_name = "at%d" % id_num
            self.tag_pubs.append(rospy.Publisher(pub_name, Bool, queue_size=1))

        self.rate = rospy.Rate(10)

        # Subscribe to april tag detector topic
        self.tag_detections = rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                                               callback=self.tag_detections_handler)

        while not rospy.is_shutdown():
            rospy.spin()

    def tag_detections_handler(self, msg):
        """
        Publishes boolean msg on topics associated with detected april tags.
        msg: AprilTagsDetectionArray object
        returns: none
        """
        rospy.loginfo("Starting over")
        for detection in msg.detections:
            id_num = detection.id[0] # Using standalone tags
            index = self.tag_ids.index(id_num)
            msg = Bool()
            msg.data = True
            rospy.loginfo("Publishing message on topic at%d" % id_num)
            self.tag_pubs[index].publish(msg)
            self.rate.sleep()

    def shutdown(self):
        """
        Runs on node shutdown.
        returns: none
        """

        rospy.loginfo("Stopping tag_handler node")


if __name__ == '__main__':

    try:
        AprilTagHandler()
    except:
        rospy.logerr("AprilTagHandler failed")
