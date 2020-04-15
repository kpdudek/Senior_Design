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
from RobotControllerClass_Ethan import RobotController


class JoyController(object):
    def __init__(self):

        # Joystick axes and buttons
        self.left_stick_val = 0
        self.left_stick_press = 0
        self.B = 0
        self.A = 0
        self.Y = 0
        self.X = 0

    def joy_callback(self,data):
        self.A = data.buttons[0]
        self.B = data.buttons[1]
        self.Y = data.buttons[3]
        self.X = data.buttons[2]

        self.left_stick_val = -1.0 * data.axes[0]
        self.left_stick_press = data.buttons[9]

 