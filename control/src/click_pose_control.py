#! /usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from math import atan2

linear_def = 0.3
angular_def = 3.5

x=y=xx=yy=0
theta = 0
pi = 3.14
flagMOVE = 0

def callback_pos(msg):
    global x,y,theta
    
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = round((theta + pi/2), 2)
    #rospy.loginfo("pos")
    #print ("Vi tri: x = ",msg.pose.pose.position.x," y = ",msg.pose.pose.position.y)

def callback_click(msg):
    global xx,yy,flagMOVE
    
    xx = round(msg.point.x, 2)
    yy = round(msg.point.y, 2)
    
    flagMOVE = 1
    
    #rospy.loginfo("click")
    #print ("Vi tri click: x = ",msg.point.x," y = ",msg.point.y)

rospy.init_node("speed_controller")

sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback_pos)
sub = rospy.Subscriber('/clicked_point', PointStamped, callback_click)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

while not rospy.is_shutdown():
	
	inc_x = xx - x
	inc_y = yy - y

	angle_to_goal = atan2(inc_y, inc_x)
	
	if flagMOVE == 1 :
		if abs(angle_to_goal - theta) > 0.3: 
			#0.1*57 = 5.7 do
			#print 'angle_to_goal - theta:'
			#print angle_to_goal - theta
			if (angle_to_goal - theta) > 0:
				speed.linear.x = 0.0
				speed.angular.z = angular_def
			elif (angle_to_goal - theta) < 0:
				speed.linear.x = 0.0
				speed.angular.z = -(angular_def)
		else:
			#print 'di thang'
			#flagMOVE = 0 
			speed.linear.x = linear_def
			speed.angular.z = 0.0
			if (inc_x*inc_x + inc_y*inc_y) < 0.1:
				flagMOVE = 0 

		pub.publish(speed)
		#r.sleep()    
