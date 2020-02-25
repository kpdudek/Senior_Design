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
  global jointAngles
  jointAngles.joint_angles = list(jointAngles.joint_angles)
  for i in range(0,6):
    jointAngles.joint_angles[i] = data.pulses[i] / 2400.0 * 2 * pi
    if (jointAngles.joint_angles[i] > 2*pi):
      jointAngles.joint_angles[i] = 0.0 + jointAngles.joint_angles[i] - 2*pi
    elif (jointAngles.joint_angles[i] < 0.0):
      jointAngles.joint_angles[i] = jointAngles.joint_angles[i] + 2*pi



def main():
  global anglesPub
  global jointAngles
  rospy.init_node('Interpreter_Node', anonymous='True')

  anglesPub = rospy.Publisher('/AR3/Control',AR3_Control, queue_size = 1)

  pulsesSub = rospy.Subscriber('/Teensy/encoders', Teensy_Pulses, pulseCallback)

  jointAngles = AR3_Control()
  rate = rospy.Rate(60)
  jointAngles.run = 1

  while not rospy.is_shutdown():
    anglesPub.publish(jointAngles)
    rate.sleep()

if __name__ == "__main__":
  main()
        


