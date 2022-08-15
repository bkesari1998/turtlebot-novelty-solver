#!/usr/bin/env python

from std_srvs.srv import Trigger
import rospy
import math

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion, quaternion_from_euler


class AprilTagLocalization(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tag_localization")
        rospy.on_shutdown(self.shutdown)

        # Initialize tf transform listener
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        rospy.loginfo("tag_localization node active")

        # Create a list of active april tag ids.
        try:
            self.positional_tags = rospy.get_param("positional_tags")
            self.reset_dist_detect_max = rospy.get_param("reset_dist_detect_max")
            self.reset_dist_detect_min = rospy.get_param("reset_dist_detect_min")
            self.reset_rot_detect = rospy.get_param("reset_rot_detect")
            self.reset_dist_thresh = rospy.get_param("reset_dist_thresh")
            self.reset_rot_thresh = rospy.get_param("reset_rot_thresh")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting tag parameters.")

        self.tags = {}
        for id_num in self.positional_tags:
            self.tags[id_num] = False


        # Initialize initial_pose publisher
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    
        self.rate = rospy.Rate(60)

        # Create service proxy to state updater
        self.confirm_state = rospy.ServiceProxy("confirm_state", Trigger)

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

        set_pose_flag = True
        tags_seen = []

        for detection in msg.detections:
            tag_id = detection.id[0]
            tags_seen.append(tag_id)

            if set_pose_flag and not self.tags[tag_id]:

                # Lookup transformation between base_footprint and detected tag
                detection_to_base_foot_tf = self.buffer.lookup_transform("at%d" % tag_id, "base_footprint", rospy.Time(0))
                dist = math.sqrt(detection_to_base_foot_tf.transform.translation.x**2 + detection_to_base_foot_tf.transform.translation.y**2)
                roll, pitch, yaw = euler_from_quaternion([detection_to_base_foot_tf.transform.quaternion.x, detection_to_base_foot_tf.transform.quaternion.y, detection_to_base_foot_tf.transform.quaternion.z, detection_to_base_foot_tf.transform.quaternion.w])

                if (self.reset_dist_detect_min <= dist <= self.reset_dist_detect_max) and abs(yaw) <= self.reset_rot_detect:

                    # Lookup transformation between tag detection and static tag frame
                    detection_to_static_tf = self.buffer.lookup_transform('at%d_' % tag_id, 'at%d' % tag_id, rospy.Time(0))

                    # Compare transforms against thresholds
                    pos_diff = math.sqrt(detection_to_static_tf.transform.translation.x**2 + detection_to_static_tf.translation.vector.z**2)
                    roll, pitch, yaw = euler_from_quaternion([detection_to_static_tf.transform.quaternion.x, detection_to_static_tf.transform.quaternion.y, detection_to_static_tf.transform.quaternion.z, detection_to_static_tf.transform.quaternion.w])
                    if pitch > math.pi: pitch -= 2*math.pi
                    
                    if self.reset_dist_thresh <= pos_diff  or abs(pitch) < self.reset_rot_thresh:
                        
                        # Get pose of base link in static frame
                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = 'at%d_' % tag_id
                        pose_stamped.header.stamp = rospy.Time.now()
                        pose_stamped.pose.position.x = detection_to_base_foot_tf.transform.translation.x
                        pose_stamped.pose.position.y = detection_to_base_foot_tf.transform.translation.y
                        pose_stamped.pose.position.z = detection_to_base_foot_tf.transform.translation.z
                        pose_stamped.pose.orientation.x = detection_to_base_foot_tf.transform.quaternion.x
                        pose_stamped.pose.orientation.y = detection_to_base_foot_tf.transform.quaternion.y
                        pose_stamped.pose.orientation.z = detection_to_base_foot_tf.transform.quaternion.z
                        pose_stamped.pose.orientation.w = detection_to_base_foot_tf.transform.quaternion.w

                        transform = self.buffer.transform(pose_stamped, "map", rospy.Duration(1))

                        # Get new initial pose
                        initialpose = PoseWithCovarianceStamped()
                        initialpose.header.frame_id = "map"
                        initialpose.header.stamp = rospy.Time.now()
                        initialpose.pose.pose = transform.pose
                        initialpose.pose.pose.position.z = 0
                        roll, pitch, yaw = euler_from_quaternion([initialpose.pose.pose.orientation.x, initialpose.pose.pose.orientation.y, initialpose.pose.pose.orientation.z, initialpose.pose.pose.orientation.w])
                        quat = quaternion_from_euler([0, 0, yaw])
                        initialpose.pose.pose.orientation.x = quat[0]
                        initialpose.pose.pose.orientation.y = quat[1]
                        initialpose.pose.pose.orientation.z = quat[2]
                        initialpose.pose.pose.orientation.w = quat[3]

                        # Set covariance
                        initialpose.pose.covariance = [0] * 36
                        initialpose.pose.covariance[0] = 0.1
                        initialpose.pose.covariance[7] = 0.1
                        initialpose.pose.covariance[35] = 0.25

                        self.pose_pub.publish(initialpose)
                        set_pose_flag = False

        for id, seen in self.tags.items():
            if id in tags_seen:
                self.tags[id] = True
            else:
                self.tags[id] = False



    def shutdown(self):
        """
        Runs on node shutdown.
        returns: none
        """

        rospy.loginfo("Stopping tag_handler node")


if __name__ == '__main__':
    AprilTagLocalization()
