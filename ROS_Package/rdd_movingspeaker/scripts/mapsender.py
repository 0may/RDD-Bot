#!/usr/bin/env python

import rospy
import socket

TCP_IP=rospy.get_param("ipmaster")
TCP_PORT=rospy.get_param("mapport")
MAP_PATH=rospy.get_param("mappath")


def initConnection()
    """Initialises the TCP-Connection """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        return sock
    except Exception as e:
        print (e)
        print ("Connection not available! check\nHOST IP=" + TCP_IP + "    PORT=" + TCP_PORT)
        return 0

def callback(data):
    """Sends the mapfile that is located in the given mappath to the ipmaster-address"""
    sock = initConnection()
    try:
        if sock:
            mapfile = open(MAP_PATH, 'rb')
            while(mapfile):
                sock.send(mapfile)
                mapfile = f.read(1024)
            sock.close
    except Exception as e:
        print (e)
        print (" File not found or wrong format!! \nCheck path="+ MAP_PATH)

def readsender():
    """Gets and send the map to the given IP with the corresponding PORT"""
    rospy.init_node('mapsender', anonymous=True)
    callback("")
    rospy.Subscriber("maprefresh", String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        readsender()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
