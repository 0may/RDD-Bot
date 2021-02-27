#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


tolerance = 0.5
direction = True

# joy_state_current = 0
joy_state_old_x = 0
joy_state_old_y = 0


def clbk_cmdVel(msg):
    rospy.loginfo("Received a /cmd_vel/rev message!")
    joy_state_.append(msg)
    # rospy.loginfo("Linear Components: [%f, %f, %f]"%(joy_state_[-2].linear.x, joy_state_[-2].linear.y, joy_state_[-2].linear.z))
    
    # joy_state_old_x = msg.linear.x
    # joy_state_old_y = msg.linear.y
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))


def clbk_laser(msg):
    
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    # rospy.loginfo(regions['right'])
    take_action(regions)

def reverse():
    msg = Twist()
    if joy_state_[-2].linear.x < 0.1 or joy_state_[-2].linear.y < 0.1:
         msg.linear.x = (joy_state_[-2].linear.x + 0.1 ) * -0.5
         msg.linear.y = (joy_state_[-2].linear.y + 0.1 ) * -0.5
    else:
        # msg.linear.x = joy_state_[-2].linear.x * -0.5
        # msg.linear.y = joy_state_[-2].linear.y * -0.5
        msg.linear.x = -0.1
        msg.linear.y = -0.1
    rospy.loginfo("reverse: [%f, %f]"%(msg.linear.x, msg.linear.y))
    pub_rev.publish(msg)

def take_action(regions):
    msg = Bool()
    # linear_x = 0
    # angular_z = 0

    pause_navigation = False
    state_description = ''

    if regions['front'] > 1*tolerance and regions['fleft'] > 1*tolerance and regions['fright'] > 1*tolerance:
        state_description = 'case 1 - nothing'
        pause_navigation = False
        
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

    if pause_navigation:
        reverse()
    # msg.linear.x = linear_x
    # msg.angular.z = angular_z
    msg.data = pause_navigation
    # msg.data = False
    pub.publish(msg)

def main():
    global pub, pub_rev, joy_state_

    joy_state_ = []

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/pause_navigation',Bool,queue_size=1)
    pub_rev = rospy.Publisher('/cmd_vel/rev', Twist, queue_size=1)

    rospy.Subscriber('/evo_robot/laser_1/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/cmd_vel/joy', Twist, clbk_cmdVel)

    rospy.spin()

if __name__ == '__main__':
    main()
