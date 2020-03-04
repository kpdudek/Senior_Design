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

from Teensy.msg import Teensy_Pulses
from RobotControllerClass_Ethan import RobotController


class InterpreterNode(RobotController):
  def __init__(self):
    super(InterpreterNode,self).__init__()
    rospy.init_node('Interpreter_Node', anonymous='True')
    
    
    self.pulsesSub = rospy.Subscriber('/Teensy/encoders', Teensy_Pulses, self.pulseCallback)

  def pulseCallback(self,data):
    for i in range(0,5):
      self.AR3Control.joint_angles[i] = data.pulses[i] / 2400.0 * 2 * pi
      if (self.AR3Control.joint_angles[i] > 2*pi):
        self.AR3Control.joint_angles[i] = 0.0 + self.AR3Control.joint_angles[i] - 2*pi
      elif (self.AR3Control.joint_angles[i] < 0.0):
        self.AR3Control.joint_angles[i] = self.AR3Control.joint_angles[i] + 2*pi

    self.send_joints()



