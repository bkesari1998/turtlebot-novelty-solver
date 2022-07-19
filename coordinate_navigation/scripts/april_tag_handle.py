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
        rospy.Publisher("at1", Bool, queue_size=10)

        self.rate = rospy.Rate(10)

        # Subscribe to april tag detector topic
        self.tag_detections = rospy.Subscriber("tag_detection", AprilTagDetectionArray, callback=self.tag_detections_handler)

    def tag_detections_handler(self, msg):
        '''
        Publishes boolean msg on topics associated with detected april tags.
        msg: AprilTagsDetectionArray object
        returns: none
        '''

        for detection in msg.detections:
            index = self.tag_ids.index(detection.id)
            msg = Bool()
            msg.data = True
            self.tag_pubs[index].publish(msg)
            self.rate.sleep()
    
    def shutdown(self):
        '''
        Runs on node shutdown.
        returns: none
        '''

        rospy.loginfo("Stopping tag_handler node")

if __name__ == '__main__':
    
    try:
        AprilTagHandler()
    except:
        rospy.logerr("AprilTagHandler failed")