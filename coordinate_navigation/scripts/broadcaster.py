#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('broadcaster')

    rate = rospy.Rate(100)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "at0_"
    t.transform.translation.x = 23.13571
    t.transform.translation.y = 18.01429
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(4.71238, 0, 1.815)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]


    while not rospy.is_shutdown():
        br.sendTransform(t)
        rate.sleep()
