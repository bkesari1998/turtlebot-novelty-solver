#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
from tf import TransformListener

def transform_to_tag_frame(camera_frame_pose):

    tag_frame_pose = geometry_msgs.msg.PoseStamped()

    # Header
    tag_frame_pose.header.stamp = rospy.Time.now()
    tag_frame_pose.header.frame_id = 'base_link'

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
    rospy.sleep(1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        listener.waitForTransform("/map", "/base_link", now, rospy.Duration(5))
        if(listener.canTransform('/map', '/base_link', now)):
                print('can transform')
                mpose = PoseStamped()
                
                mpose.pose.position.x = 1
                mpose.pose.position.y = 0
                mpose.pose.position.z = 0
                
                mpose.pose.orientation.x = 0
                mpose.pose.orientation.y = 0
                mpose.pose.orientation.z = 0
                mpose.pose.orientation.w = 0
                
                mpose.header.frame_id = '/base_link'
                mpose.header.stamp = now
                mpose_transf = listener.transformPose('/map',mpose)
                print(mpose_transf)

        rate.sleep()

