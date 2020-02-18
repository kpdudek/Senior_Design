#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from Teensy.msg import Teensy_Angles

global anglesSub

def getAngles(data):
  angles = [data.j1_angle, data.j2_angle, data.j3_angle, data.j4_angle, data.j5_angle, data.j6_angle]
  print(angles)

def main():
  rospy.init_node('Recorder', anonymous='True')

  anglesSub = rospy.Subscriber('/Interpreter/Publisher/Angles', Teensy_Angles, getAngles)

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()


if __name__ == "__main__":
  main()
