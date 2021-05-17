#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

def callback_pos(msg):
	rospy.loginfo("login subnode_pose")
	print "x_pos = ", msg.pose.pose.position.x
	print "y_pos = ", msg.pose.pose.position.y

def callback_click(msg):
	rospy.loginfo("login subnode_point")
	print "x_click = ", msg.point.x
	print "y_click = ", msg.point.y
	
rospy.init_node('values')
sub_pos = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback_pos)
sub_click = rospy.Subscriber('/clicked_point', PointStamped, callback_click)

rospy.spin()	
