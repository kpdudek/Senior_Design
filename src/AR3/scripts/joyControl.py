#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

global angle
global j1

angle = 0
j1 = Float64()

def joyCallback(data):
    global angle
    global j1
    global j1_pub

    buttonA = data.buttons[0]
    buttonB = data.buttons[1]
    print([buttonA,buttonB])

    if buttonA == 1:
        angle = angle + (pi/4)

        if angle < 0:
            angle = (2*pi) + angle
        elif angle >= 2*pi:
            angle = angle - 2*pi

        j1.data = angle
        j1_pub.publish(j1)

    elif buttonB == 1:
        angle = angle - (pi/4)

        if angle < 0:
            angle = (2*pi) + angle
        elif angle >= 2*pi:
            angle = angle - 2*pi

        j1.data = angle
        j1_pub.publish(j1)
    
    

def main():
    rospy.init_node('Joint_Control', anonymous='True')
    global j1_pub

    j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
    j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
    j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
    j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)

    xbox = rospy.Subscriber('/joy', Joy, joyCallback)
    # rate = rospy.Rate(100)
    
    
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()

    while not rospy.is_shutdown():
        t = time.time()
        # rospy.signal_shutdown()




if __name__ == "__main__":
	main()
