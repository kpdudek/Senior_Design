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


class RobotController(object):
    def __init__(self):
        # ROS stuff
        self.AR3Control = AR3_Control()
        self.AR3Feedback = AR3_Feedback()

        self.AR3ControlPub = rospy.Publisher('/AR3/Control', AR3_Control, queue_size = 1)
        self.AR3FeedbackSub = rospy.Subscriber('/AR3/Feedback', AR3_Feedback, self.AR3FeedbackCallback)

    def AR3FeedbackCallback(self,data):
        data.joint_angles = list(data.joint_angles)
        self.AR3Feedback.joint_angles = data.joint_angles
        self.AR3Feedback.eStop = data.eStop
        self.AR3Feedback.homed = data.homed
        self.AR3Feedback.running = data.running
        self.AR3Feedback.resting = data.resting

    def send_joints(self):
        self.AR3ControlPub.publish(self.AR3Control)

    def clc(self):
        print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
        print('AR3@Controller:$ '),
    