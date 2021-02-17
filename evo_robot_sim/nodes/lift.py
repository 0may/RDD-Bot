#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetJointProperties

from std_msgs.msg import Int8
from gazebo_msgs.msg import ODEJointProperties

g_set_jnt_prbs = None
g_last_vel = None

def callback(liftdata):

    print(liftdata)

    global g_set_jnt_prbs
    global g_last_vel

    prbs = ODEJointProperties()
    prbs.vel = [0]
    prbs.fmax = [10]

    if(liftdata.data < 0):
        prbs.vel = [-1.0]
    elif(liftdata.data > 0):
        prbs.vel = [+1.0]
    else:
        prbs.vel = [0.0]

    if(prbs.vel[0] != g_last_vel):    
        res = g_set_jnt_prbs('front_pndlm_link_2_fr_lift_link', prbs)
        res = g_set_jnt_prbs('front_pndlm_link_2_fl_lift_link', prbs)
        res = g_set_jnt_prbs('back_pndlm_link_2_br_lift_link', prbs)
        res = g_set_jnt_prbs('back_pndlm_link_2_bl_lift_link', prbs)
        g_last_vel = prbs.vel[0]
    
def lift_node():

    global g_set_jnt_prbs

    rospy.init_node('lift_node', anonymous=False)

    rospy.wait_for_service('/gazebo/set_joint_properties')
    g_set_jnt_prbs = rospy.ServiceProxy('/gazebo/set_joint_properties', SetJointProperties)

    # init joints
    prbs = ODEJointProperties()
    prbs.vel = [0]
    prbs.fmax = [10]
    res = g_set_jnt_prbs('front_pndlm_link_2_fr_lift_link', prbs)
    res = g_set_jnt_prbs('front_pndlm_link_2_fl_lift_link', prbs)
    res = g_set_jnt_prbs('back_pndlm_link_2_br_lift_link', prbs)
    res = g_set_jnt_prbs('back_pndlm_link_2_bl_lift_link', prbs)

    rospy.Subscriber("cmd_lift", Int8, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    lift_node()




