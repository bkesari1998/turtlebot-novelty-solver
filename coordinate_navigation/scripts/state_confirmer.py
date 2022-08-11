#!/usr/bin/env python
import rospy
import math

from tf.transformations import euler_from_quaternion

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_srvs.srv import Empty

from shapely.geometry import Point
from shapely.geometry import Polygon


class StateConfirmer(object):

    def __init__(self):

        # Initialize node
        rospy.init_node("state_confirmer")
        rospy.loginfo("state_confirmer node active")

        # Initialize service
        self.state_conf = rospy.Service("confirm_state", Empty, self.confirm_state)
        rospy.loginfo("confirm_state service active")

        # Get state_conf tag param
        try:
            self.state_tags = rospy.get_param("state_tags")
            self.agent_state_confirmation = rospy.get_param("agent_state_confirmation")
            self.param_boundaries = rospy.get_param("boundaries")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError
        
        self.boundaries = {}
        for boundary_name, boundary_edges in self.param_boundaries.items():
            edges = []
            for edge in boundary_edges:
                edges.append((edge[0], edge[1]))
            polygon = Polygon(edges)
            self.boundaries[boundary_name] = polygon

        rospy.loginfo("spinning")
        while not rospy.is_shutdown():
            rospy.spin()

    def confirm_state(self, req):
        rospy.loginfo("In handler")
        # Wait for apriltag detections msg
        tag_detections = []
        try:
            rospy.loginfo("Trying to get tag detections")
            tag_detections = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, rospy.Duration(1))
        except rospy.ROSException:
            rospy.logwarn("Did not recieve 'tag_detections' message")

        # Wait for odom msg
        try:
            odom_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, rospy.Duration(1))
        except rospy.ROSException:
            rospy.logwarn("Did not recive pose")
            return

        # Use boundaries to set agent "at"
        point = Point(odom_pose.position.x, odom_pose.position.y)
        for boundary_name, boundary in self.boundaries:
            if boundary.contains(point):
                rospy.set_param("agents/turtlebot/at", boundary_name)
                rospy.set_param("agents/turtlebot/facing", "%s_wall" % boundary_name)

        # Loop through tag_detections
        for detection in tag_detections:

            # Get tag id
            tag_id = str(detection.id[0])
            tag_in_camera_frame = detection.pose.pose.pose
            dist = math.sqrt(tag_in_camera_frame.position.x**2 + tag_in_camera_frame.position.y**2 + tag_in_camera_frame.position.z**2)
            _, rot, _ = euler_from_quaternion([tag_in_camera_frame.orientation.x, tag_in_camera_frame.orientation.y, tag_in_camera_frame.orientation.z, tag_in_camera_frame.orientation.w])

            # Set agent state
            try:
                state_value = self.agent_state_confirmation[tag_id]
                if type(state_value["range"]["distance"]) == list:
                    if dist >= state_value["range"]["distance"][0] and dist < state_value["range"]["distance"][1] and abs(rot) < state_value["range"]["orientation"]:
                        for key, value in state_value["state"]:
                            if key == "agent":
                                continue
                            rospy.loginfo("setting param")
                            rospy.set_param("agents/%s/%s" % (state_value["state"]["agent"], key), [value])

                else:
                     if dist < state_value["range"]["distance"] and abs(rot) < state_value["range"]["orientation"]:
                        for key, value in state_value["state"]:
                            if key == "agent":
                                continue
                            rospy.loginfo("setting param")
                            rospy.set_param("agents/%s/%s" % (state_value["state"]["agent"], key), [value])


            except ValueError:

                # Set object state
                try:
                    state_value = self.object_state_confirmation[tag_id]
                    if type(state_value["range"]["distance"]) == list:
                        if dist >= state_value["range"]["distance"][0] and dist < state_value["range"]["distance"][1] and abs(rot) < state_value["range"]["orientation"]:
                            for key, value in state_value["state"]:
                                if key == "object":
                                    continue

                                rospy.set_param("objects/%s/%s" % (state_value["state"]["object"], key), [value])

                    else:
                        if dist < state_value["range"]["distance"] and abs(rot) < state_value["range"]["orientation"]:
                            for key, value in state_value["state"]:
                                if key == "object":
                                    continue

                                rospy.set_param("objects/%s/%s" % (state_value["state"]["objects"], key), [value])


                except ValueError:
                    pass

        
        
if __name__ == "__main__":
    StateConfirmer()