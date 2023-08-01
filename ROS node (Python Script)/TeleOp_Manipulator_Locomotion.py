#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import math
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import time

buttons = [0,0,0,0,0,0,0,0,0,0,0]
axis = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
loco_mode = True
Px, Py, Pz = 27, 0, 22  # Initial end-effector position
RR,FR,RL,FL = [None,None,None,None] # wheels
Theta1, Theta2, Theta3, Theta4, gripper = [-5, 27, 9, -45, 45]
slow = False
speed = 5

def joy_callback(data):
    global buttons, axis
    buttons = data.buttons
    axis = data.axes
    axis = np.asfarray(axis)
    buttons = np.asarray(buttons)

def inv_kin(Px=27, Py=0, Pz=10):
    l1 = 7
    l2 = 16
    l3 = 15
    # Theta 1
    Theta1 = math.degrees(math.atan2(Py, Px))

    # Theta 2print("Orient CCW")
    f = math.sqrt(Px**2 + (Pz - l1)**2)
    alpha = math.degrees(math.acos((f**2 + l2**2 - l3**2) / (2*f*l2)))
    beta = math.degrees(math.atan2(Pz-5, Px))
    Theta2 = beta - alpha

    # Theta 3
    c3 = (f**2 - l2**2 - l3**2) / (2*l2*l3)
    s3 = math.sqrt(1 - c3**2)
    Theta3 = math.degrees(math.atan2(s3, c3))
    return Theta1, Theta2, Theta3

def locomotion():
    global buttons, axis, RR,FR,RL,FL, speed
    # print("locomotion")
    # print(axis[5])
    RT = axis[5]
    LT = axis[2]
    L_stick = axis[0]
    up_down_key = axis[7]

    if(up_down_key == 1.0): #up D-pad
        speed+=1
        print("speed: " ,speed)
        if speed >=9:
            speed = 9
        time.sleep(0.5)
    if(up_down_key == -1.0): #down D-pad
        speed -=1
        print("speed: " ,speed)
        if speed <= 2:
            speed = 2
        time.sleep(0.5)

    if(RT <= -0.5):
        print("moving forward")
        RL =  90 + (speed * 10)
        FL =  90 + (speed * 10)
        FR =  90 - (speed * 10)
        RR =  90 - (speed * 10)
    elif(LT <= -0.5):
        print("moving backward")
        RL = 90 - (speed * 10)
        FL = 90 - (speed * 10)
        FR = 90 + (speed * 10)
        RR = 90 + (speed * 10)
    elif L_stick >=1.0:
        RL =  90 - (speed * 10)
        FL =  90 - (speed * 10)
        FR =  90 - (speed * 10)
        RR =  90 - (speed * 10)
    elif L_stick <=-1.0:
        RL =   90 + (speed * 10)
        FL =   90 + (speed * 10)
        FR =   90 + (speed * 10)
        RR =   90 + (speed * 10)
    else:
        RL = 90 
        FL = 90
        FR = 90
        RR = 90

def xbox360_controller():
    global buttons
    global loco_mode
    global RR,FR,RL,FL, Px, Py, Pz
    global Theta1, Theta2, Theta3, Theta4
    msg = None
    print(buttons)
    rospy.init_node('xbox360_controller', anonymous=True)
    rospy.Subscriber("joy", Joy, joy_callback)
    pub = rospy.Publisher('servo', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(30)  # 10hz
    
    while not rospy.is_shutdown():

        power = buttons[8]
        if(power==1):
            time.sleep(1)
            if not loco_mode:
                print("Switching LocoMode")
                loco_mode=True
            else:
                print("Switching Manipulator mode")
                loco_mode=False

        if not loco_mode:
            manipulator()
        else:
            locomotion()
        msg = Float32MultiArray()
        scale = 180 / 240
        servo_offset1, servo_offset2, servo_offset3, servo_offset4 = 135, 55, 55, 135
        msg.data = [
            RL,
            FL,
            FR,
            RR, 
            (scale * (Theta1 + servo_offset1)),
            (scale * (Theta2 + servo_offset2)),
            (scale * (Theta3 + servo_offset3)),
            (scale * (Theta4 + servo_offset4)),
            ((gripper))
            ]
        if msg is not None:
            pub.publish(msg)
        rate.sleep()

def manipulator():
    global Px, Py, Pz
    global Theta4, Theta1, Theta2, Theta3, gripper
    # print(Theta1, Theta2, Theta3)
    increment_step_size = 0.25
    if buttons[4] == 1: #LB
        Pz = Pz - increment_step_size
        print("Reducing Pz:", Pz)
        # time.sleep(0.05)
    elif buttons[5] == 1: #RB
        Pz = Pz + increment_step_size
        print("Increasing Pz:", Pz)
        # time.sleep(0.05)
    if buttons[0] == 1: #A
        Px = Px - increment_step_size
        print("Reducing Px:", Px)
        # buttons[0] = 0
    elif buttons[3] == 1: #Y
        Px = Px + increment_step_size
        print("Increasing Px:", Px)
        # buttons[3] = 0
    if buttons[1] == 1: #B
        Py = Py - increment_step_size
        print("Reducing Py:", Py)
        # buttons[2] = 0
    elif buttons[2] == 1: #X
        Py = Py + increment_step_size
        print("Increasing Py:", Py)
        # buttons[1] = 0
    if buttons[9] == 1: #Left click
        if gripper==45:
            print("Engaging gripper")
            gripper=85
            time.sleep(0.5)
        elif gripper==85:
            print("Disengaging gripper")
            gripper=45
            time.sleep(0.5)
    elif buttons[10] == 1: #Right click
        Px, Py, Pz = 27, 0, 22
        print("Going Home")
    if buttons[6] == 1: #Orient Ccw - back
        Theta4 = Theta4 - 1
        print("Orient CCW: ", Theta4)
    elif buttons[7] == 1: #Orient Cw - start
        Theta4 = Theta4 + 1
        print("Orient CW: ", Theta4)
    else:
        pass
    try:
        Theta1, Theta2, Theta3 = inv_kin(Px=Px, Py=Py, Pz=Pz)
        # return Theta1, Theta2, Theta3, Theta4
    except:
        pass
        print("No Solution")

if __name__ == '__main__':
    try:
        xbox360_controller()
    except rospy.ROSInterruptException:
        pass
