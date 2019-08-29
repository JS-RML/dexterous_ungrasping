#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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

   

if __name__ == '__main__':
    try:
        rospy.init_node('visualizer')
        visualizer('visualizer', 'sphere', 0.2, [0,0,1])
    except rospy.ROSInterruptException: pass