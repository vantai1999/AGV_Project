#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

def callback(msg):
	rospy.loginfo("login subnode_point")
	print "x_click = ", msg.point.x
	print "y_click = ", msg.point.y
	
rospy.init_node('click_values')
sub = rospy.Subscriber('/clicked_point', PointStamped, callback)
rospy.spin()
