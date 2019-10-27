#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64

def encdrCB_1(data):
    global J1, J1_prev
    if data.data > J1_prev:
        J1.data += 1
    elif data.data < J1_prev:
        J1.data -= 1
    else:
        J1.data = J1.data

def encdrCB_2(data):
    global J2, J2_prev
    if data.data > J2_prev:
        J2.data += 1
    elif data.data < J2_prev:
        J2.data -= 1
    else:
        J2.data = J2.data

def encdrCB_3(data):
    global J3, J3_prev
    if data.data > J3_prev:
        J3.data += 1
    elif data.data < J3_prev:
        J3.data -= 1
    else:
        J3.data = J3.data

def encdrCB_4(data):
    global J4, J4_prev
    if data.data > J4_prev:
        J4.data += 1
    elif data.data < J4_prev:
        J4.data -= 1
    else:
        J4.data = J4.data


def main():
    rospy.init_node('Joint_Control', anonymous='True')

    J4_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 10)
    J1_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 10)
    J2_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 10)
    J3_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 10)

    J4_sub = rospy.Subscriber('/Arduino/01/encoder1/val',Float64,encdrCB_4)
    j1_sub = rospy.Subscriber('/Arduino/01/encoder2/val',Float64,encdrCB_1)
    j2_sub = rospy.Subscriber('/Arduino/01/encoder3/val',Float64,encdrCB_2)
    j3_sub = rospy.Subscriber('/Arduino/01/encoder4/val',Float64,encdrCB_3)

    rate = rospy.Rate(60)

    global J4, J1 , J2, J3, J4_prev, J1_prev, J2_prev, J3_prev
    J4 = Float64()
    J1 = Float64()
    J2 = Float64()
    J3 = Float64()

    J4_prev = 0
    J1_prev = 0
    J2_prev = 0
    J3_prev = 0

    val = 0
    while not rospy.is_shutdown():

        print([J4.data,J1.data,J2.data,J3.data])
        J4_pub.publish(J4)
        J1_pub.publish(J1)
        J2_pub.publish(J2)
        J3_pub.publish(J3)
        J4_prev = J4.data
        J1_prev = J1.data
        J2_prev = J2.data
        J3_prev = J3.data
        rate.sleep()

if __name__ == "__main__":
    main()