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

def main():
    rospy.init_node('Joint_Control', anonymous='True')

    J_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 10)
    J2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 10)

    j_sub = rospy.Subscriber('/Arduino/01/encoder1/val',Float64,encdrCB_1)
    j2_sub = rospy.Subscriber('/Arduino/01/encoder2/val',Float64,encdrCB_2)

    rate = rospy.Rate(30)

    global J1, J2 , J1_prev, J2_prev
    J1 = Float64()
    J2 = Float64()

    J1_prev = 0
    J2_prev = 0
    val = 0
    while not rospy.is_shutdown():

        print([J1.data,J2.data])
        J_pub.publish(J1)
        J2_pub.publish(J2)
        J1_prev = J1.data
        J2_prev = J2.data
        rate.sleep()

if __name__ == "__main__":
    main()


