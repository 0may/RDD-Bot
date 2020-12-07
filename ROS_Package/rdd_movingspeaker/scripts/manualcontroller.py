#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rdd_movingspeaker.msg import manualcontrol, midiconfig
#midiConfig = is set to true if MIDI-Config is available and in right format
midiConfig=False
#manualcontrol_publisher = global handle for publishing msgs to cmd_vel
manualcontrol_publisher=0
#control = dict that describes the current cmd_vel of the robot
# [[x,y,z][α,β,γ]]
control=[[0,0,0],[0,0,0]]
#CHANNEL_ROBOT = MIDI-CHANNEL over which robot is communicating to the GUI
CHANNEL_ROBOT=rospy.get_param("channel")
#prescaler = used to calculate the maximum time for the alive signal
prescaler=50
#alive = robot is alive until alive equals the value of the prescaler
alive=prescaler+1

def callback_midimanual(data):
    """Calls function for moving the robot"""
    global alive
    print(data.mode)
    print(data.channel, "channel")
    try:
        if midiConfig:
            alive=1
            if (data.mode == 7): # mode for alive signal
                write_pub_vel_msg(control)
                return
            if (data.channel != CHANNEL_ROBOT):
                return
            #calls movement function that corresponds to the given pitch of the current data
            options[(data.pitch)](data)
            write_pub_vel_msg(control)
            print("got befehl")
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
             data.speaker_speed: speaker_speed,}

def listener():
    """Gets midi commands to execute"""
    global alive
    global control
    global manualcontrol_publisher
    rospy.init_node("manualcontroller", anonymous=True)
    rospy.Subscriber("midimanual", manualcontrol, callback_midimanual)
    rospy.Subscriber("manualinstructions", midiconfig, callback_instructions)
    manualcontrol_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
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
    print(vel_msg)
    manualcontrol_publisher.publish(vel_msg)
    
def calculate_speed(data, i1, i2, direction):
    """Calculates the speed and direction"""
    #i1 = 0 for linear speed
    #i1 = 1 for angular speed
    #i2 = 0,1,2 for x,y,z (linear) or α,β,γ (angular)
    #direction = +1 if moving forward
    #direction = -1 if moving backward
    global control
    #mode = 1 equals to note on msg (MIDI) --> speed is calculated from velocity (MIDI)
    #max speed is 1.0
    if (data.mode == 1):
        speed = (direction/127.0)*data.velocity
    #mode = 0 equals to note off msg (MIDI) --> speed is set to zero
    if (data.mode == 0):
        speed = 0
    control[i1][i2] = speed
    write_pub_vel_msg(control)
    
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
    calculate_speed(data, 0, 0, 0)
    print(data)

def speaker_down(data):
    print("move_speed")
    calculate_speed(data, 0, 0, 0)
    print(data)

def move_speed(data):
    print("rotate_speed")
    calculate_speed(data, 0, 0, 0)
    print(data)

def rotate_speed(data):
    print("rotate_speed")
    calculate_speed(data, 0, 0, 0)
    print(data)

def speaker_speed(data):
    print("speaker_speed")
    calculate_speed(data, 0, 0, 0)
    print(data)

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
