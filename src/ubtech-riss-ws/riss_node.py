#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy

def joy_cb(msg):
    pass


if __name__ == '__main__':
    rospy.init_node('riss_node')

    _sub = rospy.Subscriber('joy', Joy, joy_cb)

    rospy.spin()
