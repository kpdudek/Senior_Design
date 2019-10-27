#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64

def encdrCB_0(data):
    global J0, J0_prev
    if data.data > J0_prev:
        J0.data += 1
    elif data.data < J0_prev:
        J0.data -= 1
    else:
        J0.data = J0.data

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


def main():
    rospy.init_node('Joint_Control', anonymous='True')

    J0_pub = rospy.Publisher('/rrbot/joint0_position_controller/command', Float64, queue_size = 10)
    J1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 10)
    J2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 10)
    J3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 10)

    j0_sub = rospy.Subscriber('/Arduino/01/encoder0/val',Float64,encdrCB_0)
    j1_sub = rospy.Subscriber('/Arduino/01/encoder1/val',Float64,encdrCB_1)
    j2_sub = rospy.Subscriber('/Arduino/01/encoder2/val',Float64,encdrCB_2)
    j3_sub = rospy.Subscriber('/Arduino/01/encoder3/val',Float64,encdrCB_3)

    rate = rospy.Rate(60)

    global J0, J1 , J2, J3, J0_prev, J1_prev, J2_prev, J3_prev
    J0 = Float64()
    J1 = Float64()
    J2 = Float64()
    J3 = Float64()

    J0_prev = 0
    J1_prev = 0
    J2_prev = 0
    J3_prev = 0

    val = 0
    while not rospy.is_shutdown():

        print([J0.data,J1.data,J2.data,J3.data])
        J0_pub.publish(J0)
        J1_pub.publish(J1)
        J2_pub.publish(J2)
        J3_pub.publish(J3)
        J0_prev = J0.data
        J1_prev = J1.data
        J2_prev = J2.data
        J3_prev = J3.data
        rate.sleep()

if __name__ == "__main__":
    main()