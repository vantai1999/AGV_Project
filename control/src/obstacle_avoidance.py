#! /usr/bin/env python

import rospy

linear_def = 0.3
angular_def = 4.5
range_def = 0.4

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[180:215]), 10),
        'fright': min(min(msg.ranges[216:251]), 10),
        'front':  min(min(msg.ranges[252:287]), 10),
        'fleft':  min(min(msg.ranges[288:323]), 10),
        'left':   min(min(msg.ranges[324:359]), 10),
    }

    take_action(regions)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > range_def and regions['fleft'] > range_def and regions['fright'] > range_def:
        state_description = 'case 1 - nothing'
        linear_x = linear_def
        angular_z = 0
    elif regions['front'] < range_def and regions['fleft'] > range_def and regions['fright'] > range_def:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = angular_def
    elif regions['front'] > range_def and regions['fleft'] > range_def and regions['fright'] < range_def:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = angular_def
    elif regions['front'] > range_def and regions['fleft'] < range_def and regions['fright'] > range_def:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -(angular_def)
    elif regions['front'] < range_def and regions['fleft'] > range_def and regions['fright'] < range_def:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = angular_def
    elif regions['front'] < range_def and regions['fleft'] < range_def and regions['fright'] > range_def:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -(angular_def)
    elif regions['front'] < range_def and regions['fleft'] < range_def and regions['fright'] < range_def:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = angular_def
    elif regions['front'] > range_def and regions['fleft'] < range_def and regions['fright'] < range_def:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0
        angular_z = angular_def
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
