#!/usr/bin/env python  
import rospy
import copy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg





if __name__ == '__main__':
    rospy.init_node('tf2_at0_broadcaster')

    rate = rospy.Rate(10)

    br = tf2_ros.TransformBroadcaster()
    br1 = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "at0"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    t1 = copy.deepcopy(t)
    t1.child_frame_id = "at1"

    while not rospy.is_shutdown():
        br.sendTransform([t, t1])

        rate.sleep()
