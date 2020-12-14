#!/usr/bin/env python
import rospy
import json
import socket
from rdd_movingspeaker.msg import midiconfig 
import time

#Variables of configparameters.launch
#JSONPATH = Path where MIDI Config File is located
JSONPATH=rospy.get_param("midiconfigpath")
#IP = IP of Robot
IP=rospy.get_param("Robo_IP")
#PORT = Port of Portlistener where JSON Midiconfig file is expected
PORT=rospy.get_param("json_midiconfig_PORT")

def talker():
    """Publishes the configuration for manual control"""
    pubinstructions = rospy.Publisher('manualinstructions', midiconfig, queue_size=10)
    rospy.init_node('jsonreceiver', anonymous=True)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((IP, PORT))
        sock.listen(2)
        online = True
    except Exception as e:
        print("Invalid IP and Port. Please change the parameters in the configparameters.launch file. Using local Config File from Path.")
        online = False

    while not rospy.is_shutdown():
        time.sleep(3)
        midiConfig = jsonread()
        pubinstructions.publish(midiConfig)

        if not online:
            rospy.spin()
        sc, address = sock.accept()
        #receive and write every 1024 bits to the file defined under JSONPATH until buffer is empty
        with open(JSONPATH, 'w') as f:
            l = sc.recv(1024)
            while(l):
                f.write(l)
                l = sc.recv(1024)


def jsonread():
    """Reads in json file from provided path"""
    try: 
        with open(JSONPATH) as json_file:
            data = json.load(json_file)
    except FileNotFoundError:
        print("path of jsonconfig is not valid")
        exit()
    except json.decoder.JSONDecodeError as e:
        print("Format error in json file, see:")
        print(e)
        exit()
    except Exception as e:
        print(e)
        exit()
    return translator(data)


def translator(configdict):
    """Translates json formatted info into the message object"""
    configMSG = midiconfig()
    try:
        configMSG.channel = configdict.get('channel')
        configMSG.move_forward = configdict.get('notes').get('move_forward') 
        configMSG.move_backward = configdict.get('notes').get('move_backward')
        configMSG.strafe_left = configdict.get('notes').get('strafe_left')
        configMSG.strafe_right = configdict.get('notes').get('strafe_right')
        configMSG.rotate_left = configdict.get('notes').get('rotate_left')
        configMSG.rotate_right = configdict.get('notes').get('rotate_right')
        configMSG.speaker_up = configdict.get('notes').get('speaker_up')
        configMSG.speaker_down = configdict.get('notes').get('speaker_down')
        configMSG.move_speed = configdict.get('controlchange').get('move_speed')
        configMSG.rotate_speed = configdict.get('controlchange').get('rotate_speed')
        configMSG.speaker_speed = configdict.get('controlchange').get('speaker_speed')
        configMSG.speaker_position_q1 = configdict.get('controlchange').get('speaker_position_q1')
        configMSG.speaker_position_q2 = configdict.get('controlchange').get('speaker_position_q2')
        configMSG.speaker_position_q3 = configdict.get('controlchange').get('speaker_position_q3')
        configMSG.speaker_position_q4 = configdict.get('controlchange').get('speaker_position_q4')
        configMSG.speaker_position_q4 = configdict.get('controlchange').get('speaker_position_reset')
    except Exception as e:
        print(e)
        exit()
    return configMSG
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
