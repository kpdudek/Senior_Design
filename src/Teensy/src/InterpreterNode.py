#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from Teensy.msg import Teensy_Angles
from Teensy.msg import Teensy_Pulses

global anglesPub
global pulsesSub

def pulseCallBack(data):
   Teensy_Angles jointAngles

   jointAngles.j1_angle = j1_pulses / 2400 * 2 * pi
   jointAngles.j2_angle = j2_pulses / 2400 * 2 * pi
   jointAngles.j3_angle = j3_pulses / 2400 * 2 * pi
   jointAngles.j4_angle = j4_pulses / 2400 * 2 * pi
   jointAngles.j5_angle = j5_pulses / 2400 * 2 * pi
   jointAngles.j6_angle = j6_pulses / 2400 * 2 * pi

    anglesPub.publish(jointAngles)


def main():
    rospy.init_node('Interpreter_Node', anonymous='True')

    anglesPub = rospy.Publisher('/Interpreter/Publisher/Angles',Teensy_Angles, queue_size = 1)

    pulsesSub = rospy.Subscriber('/Interpreter/Subscriber/Pulses', Teensy_Pulses, pulseCallback)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
         rospy.spin()
         rate.sleep()

if __name__ == "__main__":
    main()
        


