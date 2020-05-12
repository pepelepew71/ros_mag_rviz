#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

def make_arrow_points_marker(scale, tail, tip, idnum):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/base_link'
    m.header.stamp = rospy.Time.now()
    m.ns = 'points_arrows'
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.y = 0
    m.pose.orientation.w = 1
    m.scale = scale
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 0.3
    m.points = (tail, tip)
    return m

if __name__ == "__main__":

    rospy.init_node('marker_test')
    marker_pub = rospy.Publisher(name='marker_test', data_class=Marker, queue_size=5)

    while not rospy.is_shutdown():
        scale = Vector3(0.1, 0.4, 0)
        marker_pub.publish(make_arrow_points_marker(scale, Point(0,0,0), Point(1,0,0), 1))
        rospy.sleep(1.0)
