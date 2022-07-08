#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int64

# open waypoint name and coordinate file
waypoint_names = open("/home/mulip-zeus/catkin_ws/src/coffee-bot/coordinate_navigation/txt/waypoint_names.txt", "r+")
waypoints = open("/home/mulip-zeus/catkin_ws/src/coffee-bot/coordinate_navigation/txt/waypoints.txt", "a+")

def write_point(msg):
    global waypoint_names
    global waypoints

    point_name = waypoint_names.readline()
    waypoints.write(point_name)
    rospy.loginfo(point_name)
    rospy.loginfo(msg)
    waypoints.write("(%f, %f, %f)" % (msg.point.x, msg.point.y, 0))
    waypoints.write("\n\n")

def set_line(msg):
    waypoint_names.seek(0, 0)
    waypoints.seek(0, 0)

    for i in range(msg.data):
        waypoint_names.readline()
        for i in range(3):
            waypoints.readline()


if __name__ == "__main__":

    # init node, non-anonymous node
    rospy.init_node("set_waypoints", anonymous=False)

    # subscribe to /clicked_points topic
    points_sub = rospy.Subscriber("/clicked_point", PointStamped, callback=write_point)
    
    # subscribe to /set_line topic
    set_line_sub = rospy.Subscriber("/set_line", Int64, set_line)

    while not rospy.is_shutdown():
        rospy.spin()

waypoint_names.close()
waypoints.close()
