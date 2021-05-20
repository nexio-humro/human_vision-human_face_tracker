#!/usr/bin/env python

import math

from geometry_msgs.msg import Point32
import rospy
from std_msgs.msg import String

from face_tracker.face_tracker import FaceTracker
from std_msgs.msg import Float32

pub = rospy.Publisher('/ll_controller/servo_yaw/command', Float32, queue_size=10)
pozycja = 0

def callback(data):
    rospy.loginfo("helo")
  #  rospy.loginfo(f'Received coordinates ({data.x:.2f}, {data.y:.2f}, {data.z:.2f})')
    angle = FaceTracker().calculate_rotation_angle(data.x, data.z)
    angle_degrees = radians_to_degrees(angle)
    global pozycja
    
	
    if not math.isnan(angle_degrees):
        pozycja += angle_degrees * 0.1
	if pozycja > 60:
            pozycja = 60

        if pozycja < -60:
            pozycja =-60
    
    pub.publish(pozycja)
  #  rospy.loginfo(f'Calculated angle = {angle:.2f} radians, angle = {angle_degrees:.2f} degrees')

def radians_to_degrees(angle):
    return angle/(2*math.pi) * 360

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/human_recognition/clientPosition', Point32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
