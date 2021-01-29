#!/usr/bin/env python

import math

from geometry_msgs.msg import Point32
import rospy
from std_msgs.msg import String

from face_tracker.face_tracker import FaceTracker

def callback(data):
    rospy.loginfo(f'Received coordinates ({data.x:.2f}, {data.y:.2f}, {data.z:.2f})')
    angle = FaceTracker().calculate_rotation_angle(data.x, data.z)
    angle_degrees = radians_to_degrees(angle)
    rospy.loginfo(f'Calculated angle = {angle:.2f} radians, angle = {angle_degrees:.2f} degrees')

def radians_to_degrees(angle):
    return angle/(2*math.pi) * 360

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('topic', Point32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
