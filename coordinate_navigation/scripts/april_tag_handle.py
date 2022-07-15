#!/usr/bin/env python

import rospy
import math

from apriltag_ros.msg import AprilTagDetectionArray
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return  yaw_z # in radians

def distance(x, y):

    return math.sqrt(x**2 + y**2)

def shutdown():
    pass

def tag_detections_handler(msg):

    tag_distance = distance(msg.detections[0].pose.pose.position.x, msg.detections[0].pose.pose.position.y)
    tag_orientation = euler_from_quaternion(msg.detections[0].pose.orientation.x, msg.detections[0].pose.orientation.y, msg.detections[0].pose.orientation.z, msg.detections[0].pose.orientation.w)

    if (tag_distance < 2 and tag_orientation < math.pi / 4):

        return True

    return False

if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node("tag_detector")
    rospy.on_shutdown(shutdown)
    rospy.loginfo("tag_detector node active")

    # Subscribe to april tag detector topic
    tag_detections = rospy.Subscriber("tag_detection", AprilTagDetectionArray, callback=tag_detections_handler)