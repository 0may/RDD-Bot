#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException

g_pub_twist = None
tf_listener = None
was_moving = False
tf_printed = False

def callback(joydata):
    global g_pub_twist
    global tf_listener
    global was_moving
    global tf_printed

    if(joydata.buttons[5] or joydata.buttons[7]):        
        # differentiate between slow and fast dead man switch
        if(joydata.buttons[7]):
            speed_factor = 0.1
        else:
            speed_factor = 1.0

        # prefer the directional pad for moving in x direction
        if joydata.axes[7] != 0:
            vx_ms = joydata.axes[7] * speed_factor * 0.5
        else:
            vx_ms = joydata.axes[1] * speed_factor


        # prefer the directional pad for moving in y direction as well
        if joydata.axes[6] != 0:
            vy_ms = joydata.axes[6] * speed_factor * 0.5
        else:
            vy_ms = joydata.axes[0] * speed_factor

        # rotate with the second analogue stick
        vphi_rads = joydata.axes[3] * speed_factor

        twistdata = Twist()

        twistdata.linear.x = vx_ms
        twistdata.linear.y = vy_ms
        twistdata.angular.z = vphi_rads
        
        g_pub_twist.publish(twistdata)
        was_moving = True

    else:
        if(was_moving):
            twistdata = Twist()

            twistdata.linear.x = 0
            twistdata.linear.y = 0
            twistdata.angular.z = 0
        
            g_pub_twist.publish(twistdata)
            was_moving = False

        if(joydata.buttons[10]):
            if(not tf_printed):
                rospy.loginfo("")
                rospy.loginfo("")
                try:
                    (trans,rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                    rospy.loginfo("Printing base_footprint in map coordinates:")
                    rospy.loginfo("{:0.3f} {:0.3f} {:0.3f} {:0.3f}".format(trans[0], trans[1], rot[2], rot[3]))
                except (LookupException, ConnectivityException, ExtrapolationException):
                    rospy.logerr("No valid base_footprint received.")

                try:
                    (trans,rot) = tf_listener.lookupTransform('map', 'base_footprint_elm', rospy.Time(0))
                    rospy.loginfo("Printing base_footprint_elm in map coordinates:")
                    rospy.loginfo("{:0.3f} {:0.3f} {:0.3f} {:0.3f}".format(trans[0], trans[1], rot[2], rot[3]))
                except (LookupException, ConnectivityException, ExtrapolationException):
                    rospy.logerr("No valid base_footprint_elm received.")

            tf_printed = True
        
        else:
            tf_printed = False

 

    
def control_node():

    global g_pub_twist
    global tf_listener

    rospy.init_node('control_ps_node', anonymous=False)

    g_pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("joy_topic", Joy, callback, queue_size=1)

    tf_listener = TransformListener()

    rospy.spin()

if __name__ == '__main__':
    control_node()




