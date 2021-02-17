#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy

g_pub_lift = None

def callback(joydata):

    global g_pub_lift

    print(joydata)

    liftdata = Int8()

    if(joydata.buttons[13]):
        liftdata.data = -127
    elif(joydata.buttons[14]):
        liftdata.data = 127
    
    g_pub_lift.publish(liftdata)

    
def lift_node():

    global g_pub_lift

    rospy.init_node('lift_ps3_node', anonymous=False)

    g_pub_lift = rospy.Publisher('cmd_lift', Int8, queue_size=1)

    rospy.Subscriber("joy_topic", Joy, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    lift_node()




