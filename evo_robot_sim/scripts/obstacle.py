#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


tolerance = 1.5

def clbk_laser(msg):
    
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    rospy.loginfo(regions['right'])
    take_action(regions)

def take_action(regions):
    msg = Bool()
    linear_x = 0
    angular_z = 0

    pause_navigation = False
    state_description = ''

    if regions['front'] > 1*tolerance and regions['fleft'] > 1*tolerance and regions['fright'] > 1*tolerance:
        state_description = 'case 1 - nothing'
        pause_navigation = True
        # linear_x = 0.6
        # angular_z = 0
    elif regions['front'] <1*tolerance and regions['fleft'] >1*tolerance and regions['fright'] >1*tolerance:
        state_description = 'case 2 - front'
        pause_navigation = True
        # linear_x = 0
        # angular_z = -0.3
    elif regions['front'] >1*tolerance and regions['fleft'] >1*tolerance and regions['fright'] <1*tolerance:
        state_description = 'case 3 - fright'
        pause_navigation = True
        # linear_x = 0
        # angular_z = -0.3
    elif regions['front'] >1*tolerance and regions['fleft'] <1*tolerance and regions['fright'] >1*tolerance:
        state_description = 'case 4 - fleft'
        pause_navigation = True
        # linear_x = 0
        # angular_z = 0.3
    elif regions['front'] <1*tolerance and regions['fleft'] >1*tolerance and regions['fright'] <1*tolerance:
        state_description = 'case 5 - front and fright'
        pause_navigation = True
        # linear_x = 0
        # angular_z = -0.3
    elif regions['front'] <1*tolerance and regions['fleft'] <1*tolerance and regions['fright'] >1*tolerance:
        state_description = 'case 6 - front and fleft'
        pause_navigation = True
        # linear_x = 0
        # angular_z = 0.3
    elif regions['front'] <1*tolerance and regions['fleft'] <1*tolerance and regions['fright'] <1*tolerance:
        state_description = 'case 7 - front and fleft and fright'
        pause_navigation = True
        # linear_x = 0
        # angular_z = -0.3
    elif regions['front'] >1*tolerance and regions['fleft'] <1*tolerance and regions['fright'] <1*tolerance:
        state_description = 'case 8 - fleft and fright'
        pause_navigation = True
        # linear_x = 0
        # angular_z = -0.3
    else:
        state_description = 'unknown case'
        pause_navigation = False
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    # msg.linear.x = linear_x
    # msg.angular.z = angular_z
    msg.data = pause_navigation
    pub.publish(msg)

def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/pause_navigation',Bool,queue_size=1)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/evo_robot/laser_1/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()