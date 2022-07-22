#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs

one = PoseStamped()

trans = TransformStamped()
trans.transform.translation.x = 1

print(one)

two = tf2_geometry_msgs.do_transform_pose(one, trans)
print(two)