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
from RobotControllerClass import RobotController
from sensor_msgs.msg import Joy


class JoyController(RobotController):
    def __init__(self):
        super(JoyController,self).__init__()

        xbox = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Motion parameters
        self.joint_idx = 0
        self.ka = .005

        # Joystick axes and buttons
        self.left_stick_val = 0
        self.left_stick_press = 0
        self.B = 0
        self.A = 0
        self.Y = 0
        self.X = 0
        self.RBmp = 0
        self.LBmp = 0
        self.power = 0
    
    def set_controller_to_AR3(self):
        t = time.time()
        t_old = time.time()
        while (t-t_old) < 1:
            self.AR3Control.joint_angles = list(self.AR3Feedback.setpoint_angles)
            t = time.time()
        self.AR3Control.joint_angles = list(self.AR3Feedback.setpoint_angles)
        self.print_control()

    def home(self):
        self.AR3Control.joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]
        # self.AR3Control.run = 0
        # self.AR3Control.home = 1

    def rest(self):
        self.AR3Control.joint_angles = [0.0,1.355,1.8,0.0,5.1,0.0]

    def joy_callback(self,data):
        self.A = data.buttons[0]
        self.B = data.buttons[1]
        self.Y = data.buttons[3]
        self.X = data.buttons[2]
        self.LBmp = data.buttons[4]
        self.RBmp = data.buttons[5]
        self.power = data.buttons[8]

        self.left_stick_val = -1.0 * data.axes[0]
        self.left_stick_press = data.buttons[9]

        # Increment the joint being moved based upon left stick press
        if self.A == 1:
            self.joint_idx += 1
            if self.joint_idx > 5:
                self.joint_idx = 0
            # self.print_control()

        elif self.B == 1:
            self.joint_idx -= 1
            if self.joint_idx < 0:
                self.joint_idx = 5
            # self.print_control()

        if self.Y == 1:
            self.home()
        
        elif self.X == 1:
            self.rest()

        elif self.LBmp == 1:
            self.AR3Control.close_gripper = 0
        
        elif self.RBmp == 1:
            self.AR3Control.close_gripper = 1

        elif self.power == 1:
            if self.AR3Control.run == 0:
                self.AR3Control.run = 1
            else:
                self.AR3Control.run = 0


    def stick_move(self):
        # Modify one joint angle based upon
        self.AR3Control.joint_angles = list(self.AR3Control.joint_angles)
        self.AR3Control.joint_angles[self.joint_idx] = self.AR3Control.joint_angles[self.joint_idx] + (self.left_stick_val*self.ka)

        # Convert to global angle
        if self.AR3Control.joint_angles[self.joint_idx] < 0:
            self.AR3Control.joint_angles[self.joint_idx] = (2*pi) + self.AR3Control.joint_angles[self.joint_idx]
        elif self.AR3Control.joint_angles[self.joint_idx] >= 2*pi:
            self.AR3Control.joint_angles[self.joint_idx] = self.AR3Control.joint_angles[self.joint_idx] - 2*pi
    
    def print_control(self):
        self.clc()
        print('Moving joint: [{}]'.format(self.joint_idx+1))
        print('Control Enabled: {}'.format(self.AR3Control.run))