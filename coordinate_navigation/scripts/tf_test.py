#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray

def transform_to_tag_frame(camera_frame_pose):

    tag_frame_pose = geometry_msgs.msg.PoseWithCovarianceStamped()

    # Header
    tag_frame_pose.header.stamp = rospy.Time.now()
    tag_frame_pose.header.frame_id = 'map'

    # Position
    tag_frame_pose.pose.pose.position.x = -camera_frame_pose.pose.pose.position.x
    tag_frame_pose.pose.pose.position.y = 0
    tag_frame_pose.pose.pose.position.z = camera_frame_pose.pose.pose.position.z

    # Orientation
    tag_frame_pose.pose.pose.orientation.w = 1
    tag_frame_pose.pose.pose.orientation.x = 0
    tag_frame_pose.pose.pose.orientation.y = 0
    tag_frame_pose.pose.pose.orientation.z = 0

    return tag_frame_pose




if __name__ == '__main__':

    rospy.init_node("tf_test")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    detections_array = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=10)
    tag = detections_array.detections[0]
    tag_frame = "at%d" % tag.id[0]
    
    camera_frame_pose = transform_to_tag_frame(tag.pose)

    trans = tfBuffer.transform(camera_frame_pose, tag_frame, new_type=geometry_msgs.msg.PoseWithCovarianceStamped)

    print(trans)
