#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print 'values at 0 degree:'
    print msg.ranges[0]
    print 'values at 90 degree:'
    print msg.ranges[90]
    print 'values at 180 degree:'
    print msg.ranges[180]
    print 'values at 270 degree:'
    print msg.ranges[270]

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
