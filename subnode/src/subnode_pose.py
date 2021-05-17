#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
	rospy.loginfo("login subnode_pose")
	print "x_pos = ", msg.pose.pose.position.x
	print "y_pos = ", msg.pose.pose.position.y
		
rospy.init_node('pos_values')
sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback)
rospy.spin()
