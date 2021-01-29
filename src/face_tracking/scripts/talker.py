#!/usr/bin/env python

import random

from geometry_msgs.msg import Point32
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('topic', Point32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        z = random.uniform(-5.0, 5.0)
        point = Point32(x, y, z)
        rospy.loginfo(f'Sending coordinates ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})')
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
