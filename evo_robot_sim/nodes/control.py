#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

g_pub_command = {}
g_vtrans_max = 1.0 # maximal rotation of single wheel in m/s

def callback(twistdata):
    global g_pub_command

    vx_ms = twistdata.linear.x
    vy_ms = twistdata.linear.y
    vphi_rads = twistdata.angular.z

    # DAM to MMA: Conversions +/- are strange. We have to define the motor direction!
    vx_ms = -vx_ms
    vy_ms = -vy_ms
    vphi_rads = -vphi_rads
    
    wheel_separation_sum = 1.3
    wheel_diameter = 0.1
    mps_2_radps = 2.0 / wheel_diameter # (2 * pi) / (d * pi) = 2 / d

    rpm = {}

    # DAM to MBA: Where/how is limit of motors implemented?
    # DAM to MMA: Divisor 2 is not in C++ code - Why?
    rpm['fl'] = mps_2_radps * (vx_ms - vy_ms - (wheel_separation_sum * vphi_rads / 2.0))
    rpm['bl'] = mps_2_radps * (vx_ms + vy_ms - (wheel_separation_sum * vphi_rads / 2.0))
    rpm['fr'] = mps_2_radps * (vx_ms + vy_ms + (wheel_separation_sum * vphi_rads / 2.0))
    rpm['br'] = mps_2_radps * (vx_ms - vy_ms + (wheel_separation_sum * vphi_rads / 2.0))

    # DAM to MMA: Conversions +/- are strange. We have to define the motor direction!
    # invert left side
    rpm['fl'] = -rpm['fl']
    rpm['bl'] = -rpm['bl']

    current_max_radps = np.max(np.abs(rpm.values()))
    max_radps = g_vtrans_max * mps_2_radps

    if(current_max_radps > max_radps):
        scaler = max_radps / current_max_radps # scale wheel speed to max speed TODO: HACK-DAM
    else:
        scaler = 1

    print(scaler)

    for k in g_pub_command:
        g_pub_command[k].publish(rpm[k] * scaler)
 

    
def control_node():

    global g_pub_command

    rospy.init_node('control_node', anonymous=False)

    rospy.Subscriber("cmd_vel", Twist, callback, queue_size=1)

    g_pub_command['fr'] = rospy.Publisher('/evo_robot/wheel_fr_controller/command', Float64, queue_size=2)
    g_pub_command['fl'] = rospy.Publisher('/evo_robot/wheel_fl_controller/command', Float64, queue_size=2)
    g_pub_command['br'] = rospy.Publisher('/evo_robot/wheel_br_controller/command', Float64, queue_size=2)
    g_pub_command['bl'] = rospy.Publisher('/evo_robot/wheel_bl_controller/command', Float64, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    control_node()




