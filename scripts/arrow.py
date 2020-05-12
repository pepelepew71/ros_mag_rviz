#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import MagneticField

def get_marker(frame_id):
    marker = Marker()
    marker.action = Marker.ADD
    marker.ns = 'points_arrows'
    marker.type = Marker.ARROW
    marker.id = 1
    marker.header.frame_id = frame_id
    marker.pose.orientation.y = 0
    marker.pose.orientation.w = 1
    marker.scale = Vector3(0.1, 0.4, 0)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.3
    return marker

def callback(ros_msg):
    x = ros_msg.magnetic_field.x
    y = ros_msg.magnetic_field.y
    z = ros_msg.magnetic_field.z
    MARKER.header.stamp = rospy.Time.now()
    MARKER.points = (Point(0.0, 0.0, 0.0), Point(float(x), float(y), float(z)))
    PUB.publish(MARKER)

if __name__ == "__main__":

    rospy.init_node(name='mag_arrow', anonymous=False)

    if not rospy.has_param(param_name="~topic_mag"):
        rospy.logerr("can't find parameter ~topic_mag")
    topic_mag = rospy.get_param(param_name="~topic_mag")

    if not rospy.has_param(param_name="~frame_id"):
        rospy.logerr("can't find parameter ~frame_id")
    frame_id = rospy.get_param(param_name="~frame_id")

    PUB = rospy.Publisher(name='mag_arrow', data_class=Marker, queue_size=10)
    MARKER = get_marker(frame_id)
    rospy.Subscriber(name=topic_mag, data_class=MagneticField, callback=callback)

    rospy.spin()
