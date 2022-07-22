#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import tf


class AprilTagHandler(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tag_handler")
        rospy.on_shutdown(self.shutdown)

        # Initialize tf transform listener
        self.listener = tf.TransformListener()

        rospy.loginfo("tag_handler node active")

        # Initialize tag_detections publishers
        self.tag_in_frame = [False]
        self.tag_ids = [0] # Make this a rosparam later
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

    
    def transform_to_tag_frame(self, camera_frame_pose, id_num):

        tag_frame_pose = PoseStamped()

        # Header
        tag_frame_pose.header.stamp = rospy.Time.now()
        tag_frame_pose.header.frame_id = 'at%d_' % id_num

        # Position
        tag_frame_pose.pose.position.x = -camera_frame_pose.position.x
        tag_frame_pose.pose.position.y = 0
        tag_frame_pose.pose.position.z = camera_frame_pose.position.z

        tag_frame_pose.pose.orientation.w = 1
        tag_frame_pose.pose.orientation.x = 0
        tag_frame_pose.pose.orientation.y = 0
        tag_frame_pose.pose.orientation.z = 0

        return tag_frame_pose

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
                
                try:
                   tb_pose_in_tag_frame = self.transform_to_tag_frame(detection.pose.pose.pose, id_num)
                   tb_pose_in_map_frame = self.listener.transformPose('/map', tb_pose_in_tag_frame)
                   init_pose = PoseWithCovarianceStamped()
                   init_pose.header = tb_pose_in_map_frame.header
                   init_pose.pose.pose = tb_pose_in_map_frame.pose
                   rospy.loginfo("Publishing pose")
                   self.pose_pub.publish(init_pose)
                   self.rate.sleep()

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("Could not find transform")
                    continue

            at_msg = Bool()
            at_msg.data = True
            self.tag_pubs[index].publish(at_msg)
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
