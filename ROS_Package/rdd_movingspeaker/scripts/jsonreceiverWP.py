#!/usr/bin/env python
import rospy
import json
import socket
from rdd_movingspeaker.srv import getWaypoint,getWaypointResponse
jsonWP=dict()
JSONPATH=rospy.get_param("waypointpath")
IP=rospy.get_param("Robo_IP")
PORT=rospy.get_param("json_WP_PORT")

def getWaypointServer():
    """Initializes the Service Server to get Waypoints by number"""
    rospy.init_node('getWaypointServer', anonymous=True)
    s = rospy.Service("getWaypoint", getWaypoint, getWaypointhandle)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((IP, PORT))
        sock.listen(2)
        online = True
    except Exception as e:
        print("Invalid IP and Port. Please change the parameters in the configparameters.launch file. Using local WP File from Path.")
        online = False
    while not rospy.is_shutdown():
        jsonread()
        if not online:
            rospy.spin()
        sc, address = sock.accept()
        with open(JSONPATH, 'w') as f:
            l = sc.recv(1024)
            while(l):
                f.write(l)
                l = sc.recv(1024)
                    
def jsonread():
    """Reads in json file form provided path"""
    global jsonWP
    try:
        with open(JSONPATH) as json_file:
            data = json.load(json_file)
            jsonWP = data
    except FileNotFoundError:
        print("path of wp jsonfile is not valid")
        exit()
    except json.decoder.JSONDecoderError as e:
        print("Format error in json file, see:")
        print(e)
        exit()
    except Exception as e:
        print(e)
        exit()

def getWaypointhandle(req):
    """Service function to get Waypointinfo by number """
    global jsonWP
    wp = getWaypointResponse()
    try:
        for elem in jsonWP.get('waypoints'):
            if elem.get('n') != req.waypointnumber:
                continue
            wp.n = elem.get('n')
            wp.t = elem.get('t')
            wp.B = elem.get('B')
            wp.x = elem.get('x')
            wp.y = elem.get('y')
            wp.alpha = elem.get('alpha')
            wp.beta = elem.get('beta')
            break
    except Exception as e:
        print(e)
        exit()
    return ( wp )

if __name__ == '__main__':
    try:
        getWaypointServer()
    except rospy.ROSInterruptException:
        pass
