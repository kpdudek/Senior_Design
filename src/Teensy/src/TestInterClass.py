#!/usr/bin/env python
from Teensy.msg import AR3_Control
from AR3.msg import AR3_Feedback
import rospy
import time
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import sys

from InterpreterNodeClass import InterpreterNode

def main():

  interNode = InterpreterNode()

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == "__main__":
  main()