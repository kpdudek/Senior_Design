#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from Teensy.msg import Teensy_Pulses

def main():
  rospy.init_node('RemoteSender_Node', anonymous='True')

  pulsesPub = rospy.Publisher('/Remote/Test/Publisher/Pulses',Teensy_Pulses, queue_size = 1)

  rate = rospy.Rate(60)
  testStruct = Teensy_Pulses()
  testStruct.j1_pulses = 1000
  testStruct.j2_pulses = 2000
  testStruct.j3_pulses = 3000
  testStruct.j4_pulses = 4000
  testStruct.j5_pulses = 5000
  testStruct.j6_pulses = 6000

  while not rospy.is_shutdown():
    pulsesPub.publish(testStruct)
    rate.sleep()

if __name__ == "__main__":
  main()
