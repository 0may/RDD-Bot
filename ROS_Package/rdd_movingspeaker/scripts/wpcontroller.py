#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from rdd_movingspeaker.srv import *
from rdd_movingspeaker.msg import manualcontrol

def use_wp_service(data, wp_service):
    """Gets the parameter for the according waypoint from the service"""
    waypoint = PoseStamped()
    wpdata = wp_service(data.pitch)
    # To be done by the other project group #
    waypoint.header = ""
    waypoint.pose.position.x = wpdata.x
    waypoint.pose.position.y = wpdata.y
    waypoint.pose.position.z = 0
    waypoint.pose.orientation.x = 0
    waypoint.pose.orientation.y = 0
    waypoint.pose.orientation.z = 0
    waypoint.pose.orientation.w = 0
    # To be done by the other project group #

def subscribewaypoint(wp_service):
    """Gets the waypoint number from the midiconverter"""
    rospy.init_node('wpcontroller', anonymous=True)
    rospy.wait_for_service('getWaypoint')
    wp_service = rospy.ServiceProxy('getWaypoint', getWaypoint)
    subwp = rospy.Subscriber('midiwaypoint', manualcontrol, use_wp_service, wp_service)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        subscribewaypoint()
    except rospy.ROSInterruptExecution:
        pass
