#!/usr/bin/env python

import rospy
import socket
#UDP_IP = IP of the Host of the GUI
UDP_IP=rospy.get_param("ipmaster")
#UDP_PORT = Port of the Host of the GUI for the reception of the coordinates of the robot
UDP_PORT=rospy.get_param("coordport")

def callback(data, sock):
    """Sends the current position of the robot via UDP"""
    # data has to be converted to a readable object
    sock.sendto(data.data.encode("utf-8"), (UDP_IP, UDP_PORT)) 

def listener():
    """Gets the current position of the robot"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.init_node('positionsender', anonymous=True)
    #rospy.Subscriber("", String, callback, sock) # To be done by other project group
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
        
