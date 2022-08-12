#!/usr/bin/env python

from std_srvs.srv import Empty
import rospy
import math

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion


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
        self.confirm_state = rospy.ServiceProxy("confirm_state", Empty)

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
        tag_frame_pose.pose.position.x = camera_frame_pose.position.x
        tag_frame_pose.pose.position.y = camera_frame_pose.position.y
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

        # 1m variance in x direction
        base_footprint_pose.pose.covariance[0] = 0.1
        # 1m variance in y direction
        base_footprint_pose.pose.covariance[7] = 0.1
        # .25 radian variance in yaw axis
        base_footprint_pose.pose.covariance[35] = 0.25

        return base_footprint_pose
        

    def tag_detections_handler(self, msg):
        """
        Publishes boolean msg on topics associated with detected april tags.
        msg: AprilTagsDetectionArray object
        returns: none
        """

        # List to keep track of tags seen
        tags_seen = []
        set_pose_flag = True

        # Get odometry position
        odom_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        odom_pose = odom_pose.pose.pose

        for detection in msg.detections:

            tag_id = detection.id[0]

            try:
                self.positional_tags.index(tag_id)
            except ValueError:
                continue

            tags_seen.append(tag_id)

            tag_in_camera_frame = detection.pose.pose.pose
            dist = math.sqrt(tag_in_camera_frame.position.x**2 + tag_in_camera_frame.position.y**2 + tag_in_camera_frame.position.z**2)
            _, rot, _ = euler_from_quaternion([tag_in_camera_frame.orientation.x, tag_in_camera_frame.orientation.y, tag_in_camera_frame.orientation.z, tag_in_camera_frame.orientation.w])
            
            rospy.loginfo(dist)
            rospy.loginfo(rot)

            if rot > math.pi:
                rot -= (2*math.pi)

            # Only update if tag is in distance range
            if dist <= self.reset_dist_detect_max and dist > self.reset_dist_detect_min and \
            abs(rot) > self.reset_rot_detect:

                rospy.loginfo("Tag in range")

                # Only update pose with tag previously out of view before
                if not self.tags[tag_id]:

                    # Update tag_in_view list
                    self.tags[tag_id] = True
                    
                    # Update pose only once per set of new detections
                    if set_pose_flag:
                        
                        rospy.loginfo("Set pose")
                        set_pose_flag = False
                        
                        # Transform location of tag in camera's frame to position of camera in tag's frame
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

                        # Get position of base_footprint in map frame from april tag
                        base_foot_pose = self.pose_of_base_footprint(laser_in_map_frame, laser_to_base_transform)

                        # Only publish new position if odom position is very different from calculated pos
                        pose_diff = math.sqrt((odom_pose.position.x - base_foot_pose.pose.pose.position.x)**2 + 
                        (odom_pose.position.y - base_foot_pose.pose.pose.position.y)**2 + 
                        (odom_pose.position.z - base_foot_pose.pose.pose.position.z)**2)

                        quat_diff = quaternion_multiply([odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w],
                        quaternion_conjugate([base_foot_pose.pose.pose.orientation.x, base_foot_pose.pose.pose.orientation.y, base_foot_pose.pose.pose.orientation.z, base_foot_pose.pose.pose.orientation.w]))

                        _, _, yaw_diff = euler_from_quaternion(quat_diff)
                        rospy.loginfo(yaw_diff)
                        if yaw_diff > math.pi:
                            yaw_diff -= (2 * math.pi)
                        if pose_diff > 2 and abs(yaw_diff) > self.reset_rot_thresh: 
                            rospy.loginfo("publishing initial pose")
                            self.pose_pub.publish(base_foot_pose)
                            self.confirm_state()

        # Set tag_in_view to false for tags not seen in camera image            
        for tag_id in self.tags:

            try:
                tags_seen.index(tag_id)
            except ValueError:
                self.tags[tag_id] = False
                
        self.rate.sleep()

    def shutdown(self):
        """
        Runs on node shutdown.
        returns: none
        """

        rospy.loginfo("Stopping tag_handler node")


if __name__ == '__main__':
    AprilTagLocalization()
