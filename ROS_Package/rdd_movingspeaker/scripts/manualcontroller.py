#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rdd_movingspeaker.msg import manualcontrol, midiconfig
import os
import subprocess

#midiConfig = is set to true if MIDI-Config is available and in right format
midiConfig=False

#manualcontrol_publisher = global handle for publishing msgs to cmd_vel
manualcontrol_publisher=0

#control = dict that describes the current cmd_vel of the robot
# [[x,y,z][alpha,beta,gamma]]
control=[[0,0,0],[0,0,0]]

#CHANNEL_ROBOT = MIDI-CHANNEL over which robot is communicating to the GUI
CHANNEL_ROBOT=rospy.get_param("channel")

#prescaler = used to calculate the maximum time for the alive signal
prescaler=50
#alive = robot is alive until alive equals the value of the prescaler
alive=prescaler+1

# maximum speaker speed. see Pololu TIC 
speakerMaxSpeed = 400000000
# maximum speaker position. also determines minimum speaker position as -speakerMaxPosition
speakerMaxPosition = 32000

# velocity scalings -> they map the incoming velocities to a range of smaller velocities, i.e. [0 to maximum_velocity*scaling]
speakerVelocityScaling = 1.0
rotateVelocityScaling = 1.0
moveVelocityScaling = 1.0

isSpeakerResetting = False
isSpeakerPositioning = False



def callback_midimanual(data):
    """Calls function for moving the robot"""
    global alive
    #print(data.mode)
    #print(data.channel, "channel")
    try:
        if midiConfig:
            alive=1
            if (data.mode == 7): # alive signal (midi clock signal)
                write_pub_vel_msg(control)
                return
            if (data.channel != CHANNEL_ROBOT):
                return
            #calls movement function that corresponds to the given pitch of the current data
            options[(data.pitch)](data)
            print("MidiMsg received: mode={}  pitch={}  velo={}".format(data.mode, data.pitch, data.velocity))
            #write_pub_vel_msg(control)
            #print("got befehl")
        if not midiConfig:
            print("no midiconfig!!")
            return
    except Exception as e:
        print(e)


def callback_instructions(data):
    """Gets instructions from jsonreceiver and fills them into dataconstruct"""
    global midiConfig
    global options
    #setting variable midiConfig to true --> commands can now be sent
    midiConfig=True

    print(data)
    #load functions in dict options with the corresponding keys
    options={data.move_forward: move_forward,
             data.move_backward: move_backward,
             data.strafe_left: strafe_left,
             data.strafe_right: strafe_right,
             data.rotate_left: rotate_left,
             data.rotate_right: rotate_right,
             data.speaker_up: speaker_up,
             data.speaker_down: speaker_down,
             data.move_speed: move_speed,
             data.rotate_speed: rotate_speed,
             data.speaker_speed: speaker_speed,
             data.speaker_position_q1: speaker_position_q1,
             data.speaker_position_q2: speaker_position_q2,
             data.speaker_position_q3: speaker_position_q3,
             data.speaker_position_q4: speaker_position_q4,
             data.speaker_position_reset: speaker_position_reset,}


def listener():
    """Gets midi commands to execute"""
    global alive
    global control
    global manualcontrol_publisher
    rospy.init_node("manualcontroller", anonymous=True)
    rospy.Subscriber("midimanual", manualcontrol, callback_midimanual)
    rospy.Subscriber("manualinstructions", midiconfig, callback_instructions)
    manualcontrol_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # --- init speaker motor controller (Pololu TIC) ---
    # print status
    cmd = "ticcmd --status"
    print(cmd)
    os.system(cmd)  

    # configure controller using predefined settings from file
    cmd = "ticcmd --settings /home/nvidia/CONFIGFILES/tic_settings.txt"
    print(cmd)
    os.system(cmd)  

    # energize motor and exit safe start state
    cmd = "ticcmd --resume"
    print(cmd)
    os.system(cmd)  

    # set current position as 0-position 


    while not rospy.is_shutdown():
        if (alive == prescaler):
           stop_moving()
        alive += 1
        rospy.sleep(0.02)
        

def stop_moving():
    """Stops the movement"""
    global control
    print("OUT OF RANGE!")
    for i in range(0, 3):
        control[0][i] = 0
        control[1][i] = 0


def write_pub_vel_msg(control):
    """Publishes the control message to cmdvel"""
    global manualcontrol_publisher
    vel_msg = Twist()
    vel_msg.linear.x = control[0][0]
    vel_msg.linear.y = control[0][1]
    vel_msg.linear.z = control[0][2]
    vel_msg.angular.x = control[1][0]
    vel_msg.angular.y = control[1][1]
    vel_msg.angular.z = control[1][2]
    if vel_msg.linear.x != 0 or vel_msg.linear.y != 0 or vel_msg.linear.z != 0 or vel_msg.angular.x != 0 or vel_msg.angular.y != 0 or vel_msg.angular.z != 0:
        print(vel_msg)
    manualcontrol_publisher.publish(vel_msg)


def calculate_speed(data, i1, i2, direction):
    """Calculates the speed and direction"""
    #i1 = 0 for linear speed
    #i1 = 1 for angular speed
    #i2 = 0,1,2 for x,y,z (linear) or alpha,beta,gamma (angular)
    #direction = +1 if moving forward
    #direction = -1 if moving backward
    global control
    #mode = 1 equals to note on msg (MIDI) --> speed is calculated from velocity (MIDI)
    #max speed is 1.0
    if (data.mode == 1):
        speed = direction*data.velocity/127.0
    #mode = 0 equals to note off msg (MIDI) --> speed is set to zero
    if (data.mode == 0):
        speed = 0
    
    if (i1 == 0):
        control[i1][i2] = speed*moveVelocityScaling
    elif (i1 == 1):
        control[i1][i2] = speed*rotateVelocityScaling
    else:
        control[i1][i2] = speed

    write_pub_vel_msg(control)


def rotate_speaker_up(velocity):
    global isSpeakerPositioning
    global isSpeakerResetting

    # normalize velocity
    nvelo = velocity/127.0

    if isSpeakerResetting:
        cmd = "ticcmd --settings /home/nvidia/CONFIGFILES/tic_settings.txt"
        print(cmd)
        os.system(cmd) 
        isSpeakerResetting = False

    if isSpeakerPositioning:
        cmd = "ticcmd --max-speed " + str(speakerMaxSpeed)
        print(cmd)
        os.system(cmd)  
        isSpeakerPositioning = False       

    # send command
    cmd = "ticcmd --velocity " + str(int(nvelo*speakerMaxSpeed*speakerVelocityScaling + 0.5))
    print(cmd)
    os.system(cmd)


def rotate_speaker_down(velocity):
    global isSpeakerPositioning
    global isSpeakerResetting

    # normalize velocity
    nvelo = velocity/127.0

    if isSpeakerResetting:
        cmd = "ticcmd --settings /home/nvidia/CONFIGFILES/tic_settings.txt"
        print(cmd)
        os.system(cmd)
        isSpeakerResetting = False

    if isSpeakerPositioning:
        cmd = "ticcmd --max-speed " + str(speakerMaxSpeed)
        print(cmd)
        os.system(cmd)
        isSpeakerPositioning = False


    # send command
    cmd = "ticcmd --velocity -" + str(int(nvelo*speakerMaxSpeed*speakerVelocityScaling + 0.5))
    print(cmd)
    os.system(cmd)


def position_speaker(quadrant, qposition):
    global isSpeakerPositioning
    global isSpeakerResetting

    pos = 0 # target position
    nqpos = qposition / 127.0 # normalized quadrant position

    # quadrant = 0: angle = [0 , 90]
    # quadrant = 1: angle = [90, 180]
    # quadrant = 2: angle = [-90, 0]
    # quadrant = 3: angle = [-180, -90]
    if quadrant == 0 or quadrant == 1:
        pos = int((nqpos + quadrant) * speakerMaxPosition * 0.5 + 0.5)
    elif quadrant == 2 or quadrant == 3:
        pos = int((nqpos + (quadrant - 2)) * speakerMaxPosition * 0.5 + 0.5) * -1

    if isSpeakerResetting:
        cmd = "ticcmd --settings /home/nvidia/CONFIGFILES/tic_settings.txt"
        print(cmd)
        os.system(cmd) 
        isSpeakerResetting = False


    isSpeakerPositioning = True

    cmd = "ticcmd --max-speed " + str(int(speakerMaxSpeed*speakerVelocityScaling + 0.5))
    print(cmd)
    os.system(cmd)  

    cmd = "ticcmd --position " + str(pos)
    print(cmd)
    os.system(cmd)  


def reset_speaker_position():
    global isSpeakerResetting
    global isSpeakerPositioning

    if not isSpeakerResetting:
        isSpeakerPositioning = False
        isSpeakerResetting = True

        cmd = "ticcmd --settings /home/nvidia/CONFIGFILES/tic_settings_homing.txt"
        print(cmd)
        os.system(cmd)  

        cmd = "ticcmd --home rev"
        print(cmd)
        os.system(cmd)


def move_forward(data):
    print("move_forward")
    calculate_speed(data, 0, 0, 1)
    print(data)


def move_backward(data):
    print("move_backward")
    calculate_speed(data, 0, 0, -1)
    print(data)


def strafe_left(data):
    print("strave_left")
    calculate_speed(data, 0, 1, 1)
    print(data)


def strafe_right(data):
    print("strafe_right")
    calculate_speed(data, 0, 1, -1)
    print(data)


def rotate_left(data):
    print("rotate_left")
    calculate_speed(data, 1, 2, 1)
    print(data)


def rotate_right(data):
    print("rotate_right")
    calculate_speed(data, 1, 2, -1)
    print(data)


def speaker_up(data):
    print("speaker_up")
    rotate_speaker_up(data.velocity)
    print(data)


def speaker_down(data):
    print("speaker_down")
    rotate_speaker_down(data.velocity)
    print(data)


def speaker_position_q1(data):
    print("speaker_position_q1")
    position_speaker(0, data.velocity)


def speaker_position_q2(data):
    print("speaker_position_q2")
    position_speaker(1, data.velocity)


def speaker_position_q3(data):
    print("speaker_position_q3")
    position_speaker(2, data.velocity)


def speaker_position_q4(data):
    print("speaker_position_q4")
    position_speaker(3, data.velocity)

    
def speaker_position_reset(data):
    print("speaker_position_reset")
    reset_speaker_position()


def move_speed(data):
    global moveVelocityScaling
    print("move_speed")
    moveVelocityScaling = data.velocity/127.0
    print(data)


def rotate_speed(data):
    global rotateVelocityScaling
    print("rotate_speed")
    rotateVelocityScaling = data.velocity/127.0
    print(data)


def speaker_speed(data):
    global speakerVelocityScaling
    print("speaker_speed")
    speakerVelocityScaling = data.velocity/127.0
    print(data)


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
