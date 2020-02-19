#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from Teensy.msg import AR3_Control
from Teensy.msg import Teensy_Pulses

global anglesPub
global pulsesSub

def pulseCallback(data):
  global anglesPub
  jointAngles = AR3_Control()

  for i in range(0,5):
    jointAngles.joint_angles[i] = data.pulses[i] / 2400.0 * 2 * pi
    if (jointAngles.joint_angles[i] > 2*pi):
      jointAngles.joint_angles[i] = 0.0 + jointAngles.joint_angles[i] - 2*pi
    elif (jointAngles.joint_angles[i] < 0.0):
      jointAngles.joint_angles[i] = jointAngles.joint_angles[i] + 2*pi

  anglesPub.publish(jointAngles)


def main():
  global anglesPub
  rospy.init_node('Interpreter_Node', anonymous='True')

  anglesPub = rospy.Publisher('/Interpreter/Publisher/Angles',AR3_Control, queue_size = 1)

  pulsesSub = rospy.Subscriber('/Teensy/encoders', Teensy_Pulses, pulseCallback)

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == "__main__":
  main()
        


