#!/usr/bin/env python

from Teensy.msg import Teensy_Pulses
import RobotControllerClass_Ethan import RobotController


class InterpreterNode(RobotController):
  def __init__(self):
    rospy.init_node('Interpreter_Node', anonymous='True')
    super().__init__()
    
    self.pulsesSub = rospy.Subscriber('/Teensy/encoders', Teensy_Pulses, self.pulseCallback)

  def pulseCallback(self,data):
    for i in range(0,5):
      self.AR3_Control.joint_angles[i] = data.pulses[i] / 2400.0 * 2 * pi
      if (self.AR3_Control.joint_angles[i] > 2*pi):
        self.AR3_Control.joint_angles[i] = 0.0 + self.AR3_Control.joint_angles[i] - 2*pi
      elif (self.AR3_Control.joint_angles[i] < 0.0):
        self.AR3_Control.joint_angles[i] = self.AR3_Control.joint_angles[i] + 2*pi

    self.send_joints()



