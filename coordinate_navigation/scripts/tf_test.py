#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
from tf import TransformListener

def transform_to_tag_frame(camera_frame_pose):

    tag_frame_pose = geometry_msgs.msg.PoseStamped()

    # Header
    tag_frame_pose.header.stamp = rospy.Time.now()
    tag_frame_pose.header.frame_id = 'at0_'

    # Position
    tag_frame_pose.pose.position.x = -camera_frame_pose.pose.pose.position.x
    tag_frame_pose.pose.position.y = 0
    tag_frame_pose.pose.position.z = camera_frame_pose.pose.pose.position.z

    tag_frame_pose.pose.orientation.w = 1
    tag_frame_pose.pose.orientation.x = 0
    tag_frame_pose.pose.orientation.y = 0
    tag_frame_pose.pose.orientation.z = 0

    return tag_frame_pose




if __name__ == '__main__':

    rospy.init_node("tf_test")
    
    listener = tf.TransformListener()
    transformer = tf.TransformerROS()


    t = tf.Transformer(True, rospy.Duration(10.0))
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/at0_', rospy.Time(0))
            listener.transformPose('/map', geometry_msgs.msg.PoseStamped())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans)
        print(rot)

        rate.sleep()

