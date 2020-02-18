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

def pulseCallback(data):
  global anglesPub
  jointAngles = Teensy_Angles()

  jointAngles.j1_angle = data.j1_pulses / 2400 * 2 * pi
  jointAngles.j2_angle = data.j2_pulses / 2400 * 2 * pi
  jointAngles.j3_angle = data.j3_pulses / 2400 * 2 * pi
  jointAngles.j4_angle = data.j4_pulses / 2400 * 2 * pi
  jointAngles.j5_angle = data.j5_pulses / 2400 * 2 * pi
  jointAngles.j6_angle = data.j6_pulses / 2400 * 2 * pi

  anglesPub.publish(jointAngles)


def main():
  global anglesPub
  rospy.init_node('Interpreter_Node', anonymous='True')

  anglesPub = rospy.Publisher('/Interpreter/Publisher/Angles',Teensy_Angles, queue_size = 1)

  pulsesSub = rospy.Subscriber('/Teensy/encoders', Teensy_Pulses, pulseCallback)

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == "__main__":
  main()
        


