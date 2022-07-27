#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_about_axis


class AprilTagHandler(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("tag_handler")
        rospy.on_shutdown(self.shutdown)

        # Initialize tf transform listener
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        rospy.loginfo("tag_handler node active")

        # Create a list to keep track if a tag is in the camera's view
        self.tag_in_view = [False, False, False, False, False, False, False, False, False, False, False, False, False, False]

        # Create a list of active april tag ids.
        self.tag_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13] # Make this a rosparam later

        # Initialize a topic/publisher for each april tag being used
        # Store publishers in a list
        self.tag_pubs = []
        for id_num in self.tag_ids:
            pub_name = "at%d" % id_num
            self.tag_pubs.append(rospy.Publisher(pub_name, Float64, queue_size=1))

        # Initialize initial_pose publisher
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    
        self.rate = rospy.Rate(60)

        # Subscribe to april tag detector topic
        self.tag_detections = rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                                               callback=self.tag_detections_handler)

        while not rospy.is_shutdown():
            rospy.spin()

    
    def transform_to_tag_frame(self, camera_frame_pose, id_num):
        """
        Transforms april tag detection coordinates from location of the tag in the camera's frame to the
        location of the camera in the reference frame of the tag.
        camera_frame_pose: PoseStamped object representing the april tag in the frame of the camera
        id_num: ID number of the april tag
        returns: PoseStamped object representing the pose in the frame of the april tag
        """

        tag_frame_pose = PoseStamped()

        # Header
        tag_frame_pose.header.frame_id = 'at%d_' % id_num

        # Position
        tag_frame_pose.pose.position.x = -camera_frame_pose.position.x
        tag_frame_pose.pose.position.y = -camera_frame_pose.position.y
        tag_frame_pose.pose.position.z = -camera_frame_pose.position.z

        # Orientation
        camera_frame_quat = [camera_frame_pose.orientation.x, camera_frame_pose.orientation.y, camera_frame_pose.orientation.z, camera_frame_pose.orientation.w]
        # Flip camera frame quat 180 deg around x and y axis
        tag_frame_quat = quaternion_multiply(camera_frame_quat, [0, .894, 0, -0.44807])

        tag_frame_pose.pose.orientation.w = tag_frame_quat[0]
        tag_frame_pose.pose.orientation.x = tag_frame_quat[1]
        tag_frame_pose.pose.orientation.y = tag_frame_quat[2]
        tag_frame_pose.pose.orientation.z = tag_frame_quat[3]

        return tag_frame_pose

    def pose_of_base_footprint(self, laser_pose, transform):
        """
        Calculates position of base_footprint in map frame based on transformation from laser_link.
        laser_pose: PoseStamped object representing location of laser in map frame
        trans: translational portion of frame transform
        rot: rotational portion of frame transform 
        """

        # Header
        base_footprint_pose = PoseWithCovarianceStamped()
        base_footprint_pose.header.frame_id = 'map'
        base_footprint_pose.header.stamp = rospy.Time.now()

        # Position
        base_footprint_pose.pose.pose.position.x = laser_pose.pose.position.x + transform.transform.translation.x
        base_footprint_pose.pose.pose.position.y = laser_pose.pose.position.y + transform.transform.translation.y
        base_footprint_pose.pose.pose.position.z = laser_pose.pose.position.z + transform.transform.translation.z

        # Orientaton
        transform_quat = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        laser_quat = [laser_pose.pose.orientation.x, laser_pose.pose.orientation.y, laser_pose.pose.orientation.z, laser_pose.pose.orientation.w]
        base_foot_quat = quaternion_multiply(laser_quat, transform_quat)

        base_footprint_pose.pose.pose.orientation.x = base_foot_quat[0]
        base_footprint_pose.pose.pose.orientation.y = base_foot_quat[1]
        base_footprint_pose.pose.pose.orientation.z = base_foot_quat[2]
        base_footprint_pose.pose.pose.orientation.w = base_foot_quat[3]

        # Covariance
        base_footprint_pose.pose.covariance = [0] * 36

        # .25m variance in x direction
        base_footprint_pose.pose.covariance[0] = 0.1
        # .25m variance in y direction
        base_footprint_pose.pose.covariance[7] = 0.1
        # 1 radian variance in yaw axis
        base_footprint_pose.pose.covariance[35] = 0.25

        return base_footprint_pose
        

    def tag_detections_handler(self, msg):
        """
        Publishes boolean msg on topics associated with detected april tags.
        msg: AprilTagsDetectionArray object
        returns: none
        """

        # List to keep track of tags seen
        tags_seen_indecies = []

        # Only set initial pose once per set of new detections
        # Create flag to track
        set_pose_flag = False

        # Loop over detected april tags.
        for detection in msg.detections:


            # Get index of tag in tag_id list
            index = self.tag_ids.index(detection.id[0])
            tags_seen_indecies.append(detection.id[0])

            
            # Publish distance to detected april_tag
            distance = Float64()
            distance.data = math.sqrt(detection.pose.pose.pose.position.x ** 2 +  detection.pose.pose.pose.position.y ** 2 +  detection.pose.pose.pose.position.z ** 2)
            self.tag_pubs[index].publish(distance)
            self.rate.sleep()

            # Only update if tag is in distance range
            if distance.data <= 3.5 and distance.data >= 1.75:
                rospy.loginfo(detection.pose.pose.pose)

                # Only update pose with tag previously out of view before
                if not self.tag_in_view[index]:

                    # Update tag_in_view list
                    self.tag_in_view[index] = True
                    
                    # Update pose only once per set of new detections
                    if not set_pose_flag:
                        
                        # Transform location of tag in camera's frame to position of camera in tag's frame
                        tag_in_camera_frame = detection.pose.pose.pose
                        laser_in_tag_frame = self.transform_to_tag_frame(tag_in_camera_frame, detection.id[0])

                        # Transform location of camera to map's frame
                        now = rospy.Time.now()
                        laser_in_tag_frame.header.stamp = now
                        try:
                            laser_in_map_frame = self.buffer.transform(laser_in_tag_frame, 'map')
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            rospy.logerr("Unable to transform from frame %s to frame 'map'" % laser_in_tag_frame.header.frame_id)
                            continue

                        # Lookup transform between laser and base_footprint
                        try:
                            laser_to_base_transform = self.buffer.lookup_transform('base_footprint', 'camera_rgb_frame', rospy.Time(0))
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            rospy.logerr("No transform available between 'laser_link' and 'base_footprint'")
                            continue

                        # Get position of base_footprint in map frame
                        base_foot_pose = self.pose_of_base_footprint(laser_in_map_frame, laser_to_base_transform)

                        # Publish
                        self.pose_pub.publish(base_foot_pose)
                        self.rate.sleep()

        # Set tag_in_view to false for tags not seen in camera image            
        for tag_id in self.tag_ids:

            try:
                tags_seen_indecies.index(tag_id)
            except ValueError:
                self.tag_in_view[self.tag_ids.index(tag_id)] = False

                # Publish that tag distance is -1
                distance = Float64()
                distance.data = -1
                self.tag_pubs[self.tag_ids.index(tag_id)].publish(distance)



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