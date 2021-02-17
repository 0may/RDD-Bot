#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_conversions.posemath as tfc
import tf

g_joints = ('fl_lift_link_2_fl_rim_link','fr_lift_link_2_fr_rim_link','bl_lift_link_2_bl_rim_link','br_lift_link_2_br_rim_link')
g_last_rotations = dict(zip(g_joints, (0.0, 0.0, 0.0, 0.0)))

def callback(jntstate):
    global g_joints
    global g_last_rotations

    rotations = dict(zip(jntstate.name, jntstate.velocity))

    for k in g_joints:
        print(k)
        print(rotations[k])


    
def odometry_node():
    rospy.init_node('odometry_node', anonymous=False)

    rospy.Subscriber("/evo_robot/joint_states", JointState, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    odometry_node()



    #   // get current wheel rotation
    #   double rotation_front_left  = _motor_front_left->getRevolutions();
    #   double rotation_front_right = _motor_front_right->getRevolutions();

    #   double rotation_back_left  = _motor_back_left->getRevolutions();
    #   double rotation_back_right = _motor_back_right->getRevolutions();

    #   // dif last known rotation
    #   double rot_dif_FL = rotation_front_left - _last_rotation_front_left;
    #   double rot_dif_FR = rotation_front_right - _last_rotation_front_right;
    #   double rot_dif_BL = rotation_back_left - _last_rotation_back_left;
    #   double rot_dif_BR = rotation_back_right - _last_rotation_back_right;

    #   // save current rotation
    #   _last_rotation_front_left  = rotation_front_left;
    #   _last_rotation_front_right = rotation_front_right;
    #   _last_rotation_back_left   = rotation_back_left;
    #   _last_rotation_back_right  = rotation_back_right;

    #   // invert left side
    #   rot_dif_FL *= (-1.0);
    #   rot_dif_BL *= (-1.0);

    #   // calc pose increment
    #   MecanumPose odom;
    #   odom._x_m = _rot2m * (rot_dif_FL + rot_dif_FR + rot_dif_BL + rot_dif_BR) / 4.0;
    #   odom._y_m = _rot2m * (-rot_dif_FL + rot_dif_FR + rot_dif_BL - rot_dif_BR) / 4.0;
    #   odom._yaw_rad = _rot2m * (-rot_dif_FL + rot_dif_FR - rot_dif_BL + rot_dif_BR) / (4.0 * _wheel_separation_sum_in_m);
    #   return odom;
