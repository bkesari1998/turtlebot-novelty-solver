#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

import waypoints_dict

class AprilTagHandler(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tag_handler")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("tag_handler node active")

        # Initialize tag_detections publishers
        self.tag_in_frame = [False, False]
        self.tag_ids = [0, 1] # Make this a rosparam later
        self.tag_pubs = []
        for id_num in self.tag_ids:
            pub_name = "at%d" % id_num
            self.tag_pubs.append(rospy.Publisher(pub_name, Bool, queue_size=1))

        # Initialize initial_pose publisher
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.rate = rospy.Rate(60)

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


        tag_ids_debounce = []

        for detection in msg.detections:
            id_num = detection.id[0] # Using standalone tags
            tag_ids_debounce.append(id_num)
            index = self.tag_ids.index(id_num)

            # If april tag was not in frame before, use its relative position to reset the particle filter
            if not self.tag_in_frame[index]:
                self.tag_in_frame[index] == True

                # Reset initial pose
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()

                # Calculate relative rotation
                quaternion = np.dot(np.array(waypoints_dict.tags[index][1]), np.linalg.inv(np.array(detection.pose.pose.pose.orientation)))
                
                pose.pose.pose.position.x = waypoints_dict.tags[index][0][0] - detection.pose.pose.pose.position.x
                pose.pose.pose.position.y = waypoints_dict.tags[index][0][1] - detection.pose.pose.pose.position.y
                pose.pose.pose.orientation.z = quaternion[2]
                pose.pose.pose.orientation.w = quaternion[3]

                self.pose_pub.publish(pose)
                self.rate.sleep()
            

            msg = Bool()
            msg.data = True
            self.tag_pubs[index].publish(msg)
            self.rate.sleep()

        set_to_false = set(tag_ids_debounce).symmetric_difference(set(self.tag_ids))
        for id_num in set_to_false:
            self.tag_in_frame[self.tag_ids.index(id_num)] == False

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
