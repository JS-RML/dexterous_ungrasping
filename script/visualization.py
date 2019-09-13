#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math
import threading

def visualizer_thread(marker):
    publisher = rospy.Publisher('viz', Marker)
    while not rospy.is_shutdown():
        publisher.publish(marker)
        rospy.sleep(0.01)

def visualizer(position, marker_type, scale, id): 
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1] 
    marker.pose.position.z = position[2] 
    marker.id = id
    #marker.lifetime.secs = 1
    x = threading.Thread(target=visualizer_thread, args=(marker,))
    x.daemon = True
    x.start()

def thin_object(point1, point2, thickness, id): 
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    # marker scale
    marker.scale.x = thickness
    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = point1[0]
    first_line_point.y = point1[1]
    first_line_point.z = point1[2]
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = point2[0]
    second_line_point.y = point2[1]
    second_line_point.z = point2[2]
    marker.points.append(second_line_point)
    marker.id = id
    x = threading.Thread(target=visualizer_thread, args=(marker,))
    x.daemon = True
    x.start()

if __name__ == '__main__':
    try:
        rospy.init_node('visualizer')
        visualizer('visualizer', 'sphere', 0.2, [0,0,1])
    except rospy.ROSInterruptException: pass