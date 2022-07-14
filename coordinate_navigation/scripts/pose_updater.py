#!/usr/bin/env python

import rospy
from waypoints_dict import waypoints

from apriltag_ros.msg import AprilTagDetectionArray


def shutdown():
    pass

def tag_detections_handler(msg):
    pass

if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node("pose_updater")
    rospy.on_shutdown(shutdown)
    rospy.loginfo("pose_updator node active")

    # Subscribe to april tag detector topic
    tag_detections = rospy.Subscriber("tag_detection", AprilTagDetectionArray, callback=tag_detections_handler)