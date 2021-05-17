#! /usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from math import atan2
import time

linear_def = 0.18
angular_def = 1.8
range_def = 0.5

x=y=xx=yy=0
theta = 0
pi = 3.14
flagMOVE = flagAVOID = 0

#111111111111111111111111111111111111111111111
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
#111111111111111111111111111111111111111111111

#222222222222222222222222222222222222222222222
def clbk_laser(msg):
    global flagAVOID, regions
    regions = {
    'right':  min(min(msg.ranges[180:215]), 10),
    'fright': min(min(msg.ranges[216:251]), 10),
    'front':  min(min(msg.ranges[252:287]), 10),
    'fleft':  min(min(msg.ranges[288:323]), 10),
    'left':   min(min(msg.ranges[324:359]), 10),}
    if min(min(msg.ranges[180:359]), 10) < range_def:
	flagAVOID = 1
    else:
	flagAVOID = 0
    pub.publish(speed)

def take_action(regions):
    state_description = ''

    if regions['front'] > range_def and regions['fleft'] > range_def and regions['fright'] > range_def:
	state_description = 'case 1 - nothing'
	speed.linear.x = linear_def
	speed.angular.z = 0
    elif regions['front'] < range_def and regions['fleft'] > range_def and regions['fright'] > range_def:
	state_description = 'case 2 - front'
	speed.linear.x = 0
	speed.angular.z = angular_def
    elif regions['front'] > range_def and regions['fleft'] > range_def and regions['fright'] < range_def:
	state_description = 'case 3 - fright'
	speed.linear.x = 0
	speed.angular.z = angular_def
    elif regions['front'] > range_def and regions['fleft'] < range_def and regions['fright'] > range_def:
	state_description = 'case 4 - fleft'
	speed.linear.x = 0
	speed.angular.z = -(angular_def)
    elif regions['front'] < range_def and regions['fleft'] > range_def and regions['fright'] < range_def:
	state_description = 'case 5 - front and fright'
	speed.linear.x = 0
	speed.angular.z = angular_def
    elif regions['front'] < range_def and regions['fleft'] < range_def and regions['fright'] > range_def:
	state_description = 'case 6 - front and fleft'
	speed.linear.x = 0
	speed.angular.z = -(angular_def)
    elif regions['front'] < range_def and regions['fleft'] < range_def and regions['fright'] < range_def:
	state_description = 'case 7 - front and fleft and fright'
	speed.linear.x = 0
	speed.angular.z = angular_def
    elif regions['front'] > range_def and regions['fleft'] < range_def and regions['fright'] < range_def:
	state_description = 'case 8 - fleft and fright'
	speed.linear.x = 0
	speed.angular.z = angular_def
    else:
	state_description = 'unknown case'
	rospy.loginfo(regions)
    pub.publish(speed)
    time.sleep(0.5)
    speed.linear.x = 0.0
    speed.angular.z = 0.0


#222222222222222222222222222222222222222222222
rospy.init_node("speed_controller")

sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback_pos)
sub = rospy.Subscriber('/clicked_point', PointStamped, callback_click)
sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

while not rospy.is_shutdown():

    inc_x = xx - x
    inc_y = yy - y

    angle_to_goal = atan2(inc_y, inc_x)
    angle = angle_to_goal - theta
    if (angle > 2*pi):
	angle = angle - 2*pi
    elif (angle < -2*pi):
	angle = angle + 2*pi
    #print angle
    
    if flagMOVE == 1 :
	if flagAVOID == 1 :
	    #print 'tranh vat can'
	    take_action(regions)
	elif abs(angle) > 0.2: 
	    #0.1*57 = 5.7 do
	    #print 'angle_to_goal - theta:'
	    #print angle_to_goal - theta
	    if (angle > 0):
		#print 'quay trai'
		speed.linear.x = 0.0
		speed.angular.z = angular_def
	    elif (angle < 0):
		#print 'quay phai'
		speed.linear.x = 0.0
		speed.angular.z = -angular_def
	else:
	    #print 'di thang'
	    #flagMOVE = 0 
	    speed.linear.x = linear_def
	    speed.angular.z = 0.0
	pub.publish(speed)
	time.sleep(0.1)
	speed.linear.x = 0.0
	speed.angular.z = 0.0
	
	
    if (inc_x*inc_x + inc_y*inc_y) < 0.1:
	#print 'ok'
	if (flagMOVE == 1):
	    speed.linear.x = 0.0
	    speed.angular.z = 0.0
	    pub.publish(speed)
	    flagMOVE = 0
	
	#r.sleep()    
