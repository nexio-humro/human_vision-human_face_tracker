#!/usr/bin/env python

import math

from geometry_msgs.msg import Point32
import rospy
from std_msgs.msg import String

from face_tracker.face_tracker import FaceTracker

pub = rospy.Publisher('/controllers/head_position_controller/command', Float64, queue_size=10)
pozycja = 0

def callback(data):
  #  rospy.loginfo("helo")
  #  rospy.loginfo(f'Received coordinates ({data.x:.2f}, {data.y:.2f}, {data.z:.2f})')
    angle = FaceTracker().calculate_rotation_angle(data.x, data.z)
  #  angle_degrees = radians_to_degrees(angle)
    global pozycja
    
	
#    if not math.isnan(angle_degrees):
    if not math.isnan(angle):
     #   pozycja += angle_degrees * 0.1
	pozycja -= angle * 0.1
#	pozycja = angle

#	if pozycja > 60:
#            pozycja = 60
	if pozycja > 1.0:
            pozycja = 1.0

#        if pozycja < -60:
#            pozycja =-60
        if pozycja < -1.0:
            pozycja =-1.0
    
    pub.publish(pozycja)
  #  rospy.loginfo(f'Calculated angle = {angle:.2f} radians, angle = {angle_degrees:.2f} degrees')
    rospy.loginfo("Calculated angle = %.2f radians, pozycja = %.2f", angle, pozycja)

def radians_to_degrees(angle):
    return angle/(2*math.pi) * 360

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/human_recognition/clientPosition', Point32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
