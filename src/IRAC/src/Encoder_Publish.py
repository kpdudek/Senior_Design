#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Int16

def encdrCB_1(data):
    global J1, J1_count, encoder_count
    if data.data > J1_count:
        J1.data += encoder_count
        J1_count += 1
    elif data.data < J1_count:
        J1.data -= encoder_count
        J1_count -= 1
    else:
        J1.data = J1.data
        J1_count = J1_count

def encdrCB_2(data):
    global J2, J2_count, encoder_count
    if data.data > J2_count:
        J2.data += encoder_count
        J2_count += 1
    elif data.data < J2_count:
        J2.data -= encoder_count
        J2_count -= 1
    else:
        J2.data = J2.data
        J2_count = J2_count

def encdrCB_3(data):
    global J3, J3_count, encoder_count
    if data.data > J3_count:
        J3.data += encoder_count
        J3_count += 1
    elif data.data < J3_count:
        J3.data -= encoder_count
        J3_count -= 1
    else:
        J3.data = J3.data
        J3_count = J3_count

def encdrCB_4(data):
    global J4, J4_count, encoder_count
    #print([J4.data, data.data, J4_count, encoder_count])
    if data.data > J4_count:
        J4.data += encoder_count
        J4_count += 1
    elif data.data < J4_count:
        J4.data -= encoder_count
        J4_count -= 1
 


def main():
    rospy.init_node('Joint_Control', anonymous='True')

    global J1, J2, J3, J4, J1_count, J2_count, J3_count, J4_count, encoder_count
    encoder_count = (1.0/30.0)*2*pi

    J1 = Float64()
    J2 = Float64()
    J3 = Float64()
    J4 = Float64() 

    J1_count = 0
    J2_count = 0
    J3_count = 0
    J4_count = 0

    
    J1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 10)
    J2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 10)
    J3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 10)
    J4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 10)

    j1_sub = rospy.Subscriber('/Arduino/01/encoder1/val',Int16,encdrCB_1)
    j2_sub = rospy.Subscriber('/Arduino/01/encoder2/val',Int16,encdrCB_2)
    j3_sub = rospy.Subscriber('/Arduino/01/encoder3/val',Int16,encdrCB_3)
    J4_sub = rospy.Subscriber('/Arduino/01/encoder4/val',Int16,encdrCB_4)

    rate = rospy.Rate(60)

    val = 0
    while not rospy.is_shutdown():

        print([J1.data,J2.data,J3.data,J4.data])
        J1_pub.publish(J1)
        J2_pub.publish(J2)
        J3_pub.publish(J3)
        J4_pub.publish(J4)

        rate.sleep()

if __name__ == "__main__":
    main()