#!/usr/bin/env python
import rospy
import pygame.midi
from rdd_movingspeaker.msg import manualcontrol
MIDIPORT=rospy.get_param("midiport")

def talker(input_device):
    """Publishes the MIDI command from the MIDI device"""
    rospy.init_node('midiconverter', anonymous=True)
    #pubmaprefresh = rospy.Publisher('maprefresh', int, queue_size=10)
    pubmanual = rospy.Publisher('midimanual', manualcontrol, queue_size=10)
    pubwaypoint = rospy.Publisher('midiwaypoint', manualcontrol, queue_size=10)
    while not rospy.is_shutdown():
        if input_device.poll():
                midiobject = input_device.read(1)
                midiMSG = translator(midiobject) 
                if (midiMSG.mode in (0, 1, 3, 7)):  # 0 is noteOff, 1 is noteOn, 2 is controlChange, 7 is systemCommon (clock), 6 is pitchBend
                    pubmanual.publish(midiMSG)      #MIDI command for manual control
                elif (midiMSG.mode is 6):
                    pubwaypoint.publish(midiMSG)    #MIDI command for WP control
                #if (midiMSG.pitch == 2):             #muss noch geklaert werden
                #    pubmaprefresh.publish("")
                print(midiobject)
                      
def translator(midiobject):
    """Translates the MIDI message into a readable object"""
    try:
        midiMSG = manualcontrol()
        midituple = (midiobject[0][0][0], midiobject[0][0][1], midiobject[0][0][2], midiobject[0][0][3], midiobject[0][1])
        midiMSG.mode = (midituple[0] & 0b01110000) >> 4
        midiMSG.velocity = midituple[2]
        midiMSG.channel = midituple[0] & 0b00001111
        midiMSG.pitch = midituple[1]
        midiMSG.timestamp = midituple[4]
        if (midiMSG.mode is 6):        #Pitchbend Message
            midiMSG.pitch = midiMSG.velocity + midi.pitch << 7
    except Exception as e:
        print(e)
    return midiMSG 

def list_midi_devices():
    """Prints all available MIDI USB-Devices"""
    for n in range(pygame.midi.get_count()):
        print (n,pygame.midi.get_device_info(n))

def get_midi_device():
    """Returns the MIDI input device object"""
    midiport=MIDIPORT
    while True:
        try:
            my_input = pygame.midi.Input(int(midiport))
            break
        except:
            list_midi_devices()
            midiport = raw_input("Invalid PORT. Please insert a new PORT-number: ")
    return(my_input)
        
if __name__ == '__main__':
    try:
        pygame.midi.init()
        my_input = get_midi_device()
        talker(my_input)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
