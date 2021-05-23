#! /usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from math import atan2
import time

linear_def = 0.18
angular_def = 1.5

x=y=xx=yy=0
theta = 0
pi = 3.14
check = 0

def callback_pos(msg):
    global x,y,theta
    
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = round(theta, 2)
    #rospy.loginfo("pos")
    #print ("Vi tri: x = ",msg.pose.pose.position.x," y = ",msg.pose.pose.position.y,"theta = ",theta)

def callback_click(msg):
    global xx,yy,check
    
    xx = round(msg.point.x, 2)
    yy = round(msg.point.y, 2)
    
    check = 1
    
    #rospy.loginfo("click")
    #print ("Vi tri click: x = ",msg.point.x," y = ",msg.point.y)

rospy.init_node("speed_controller")

sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback_pos)
sub = rospy.Subscriber('/clicked_point', PointStamped, callback_click)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(1)

while not rospy.is_shutdown():
	
	inc_x = xx - x
	inc_y = yy - y

	angle_to_goal = atan2(inc_y, inc_x)
	angle = angle_to_goal - theta + pi/2
	if (angle >= 2*pi):
	    angle = angle - 2*pi
	if (angle <= 0):
	    angle = angle + 2*pi
	#print angle*57
	if check == 1 :
		if abs(angle - pi) > 0.2: 
			#0.1*57 = 5.7 do
			#print 'angle_to_goal - theta:'
			if (angle) > pi:
				#print 'quay trai'
				speed.linear.x = 0.0
				speed.angular.z = angular_def
			elif (angle) < pi:
				#print 'quay phai'
				speed.linear.x = 0.0
				speed.angular.z = -angular_def
		else:
			#print 'di thang'
			#check = 0 
			speed.linear.x = linear_def
			speed.angular.z = 0.0
			if (inc_x*inc_x + inc_y*inc_y) < 0.03:
				rospy.loginfo("Finished the goal x = %s y = %s", xx,yy)
				check = 0
		pub.publish(speed)
		time.sleep(0.1)
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		
		#r.sleep()    
