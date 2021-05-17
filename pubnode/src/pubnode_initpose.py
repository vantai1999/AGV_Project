#! /usr/bin/env python

import sys
import math
from pyquaternion import Quaternion
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

'''
def __pub_initial_position(self, x, y, theta):
	"""
	Publishing new initial position (x, y, theta) --> for localization
	:param x x-position of the robot
	:param y y-position of the robot
	:param theta theta-position of the robot
	"""
	initpose_msg = PoseWithCovarianceStamped()
	initpose_msg.header.stamp = rospy.get_rostime()
	initpose_msg.header.frame_id = "map"
	initpose_msg.pose.pose.position.x = x
	initpose_msg.pose.pose.position.y = y
	quaternion = self.__yaw_to_quat(theta)

	initpose.pose.pose.orientation.w = quaternion[0]
	initpose.pose.pose.orientation.x = quaternion[1]
	initpose.pose.pose.orientation.y = quaternion[2]
	initpose.pose.pose.orientation.z = quaternion[3]
	self.__initialpose_pub.publish(initpose_msg)
	return
'''
def yaw_to_quat(yaw):
	"""
	Computing corresponding quaternion q to angle yaw [rad]
	:param yaw
	:return: q
		"""
	q = Quaternion(axis=[0, 0, 1], angle=yaw)
	return q.elements

rospy.init_node('init_pos')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)
initpose_msg = PoseWithCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = float(sys.argv[1])
initpose_msg.pose.pose.position.y = float(sys.argv[2])
quaternion = yaw_to_quat((float(sys.argv[3])))
initpose_msg.pose.pose.orientation.x = quaternion[0]
initpose_msg.pose.pose.orientation.y = quaternion[1]
initpose_msg.pose.pose.orientation.z = quaternion[2]
initpose_msg.pose.pose.orientation.w = quaternion[3]

rospy.sleep(1)

rospy.loginfo ( "Setting initial pose")
pub.publish(initpose_msg)
print(initpose_msg)
rospy.loginfo ( "Initial pose SET")
