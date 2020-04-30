#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
import sys
import os,getopt
from Teensy.msg import AR3_Control
from AR3.msg import AR3_Feedback
from control_msgs.msg import JointControllerState

import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController

class SimulatedAR3Feedback(object):
    def __init__(self):
       
        self.AR3FeedbackPub = rospy.Publisher('/AR3/Feedback', AR3_Feedback, queue_size = 1)
        self.AR3_Feedback = AR3_Feedback()
        self.AR3_Feedback.running = 1

        self.j1_sub = rospy.Subscriber('/rrbot/joint1_position_controller/state', JointControllerState, self.joint1_callback)
        self.j2_sub = rospy.Subscriber('/rrbot/joint2_position_controller/state', JointControllerState, self.joint2_callback)
        self.j3_sub = rospy.Subscriber('/rrbot/joint3_position_controller/state', JointControllerState, self.joint3_callback)
        self.j4_sub = rospy.Subscriber('/rrbot/joint4_position_controller/state', JointControllerState, self.joint4_callback)
        self.j5_sub = rospy.Subscriber('/rrbot/joint5_position_controller/state', JointControllerState, self.joint5_callback)
        self.j6_sub = rospy.Subscriber('/rrbot/joint6_position_controller/state', JointControllerState, self.joint6_callback)

    def joint1_callback(self,data):
        self.AR3_Feedback.joint_angles[0] = -1*(data.set_point + data.error)
        self.AR3_Feedback.setpoint_angles[0] = -1*(data.set_point)

    def joint2_callback(self,data):
        self.AR3_Feedback.joint_angles[1] = data.set_point + data.error
        self.AR3_Feedback.setpoint_angles[1] = data.set_point

    def joint3_callback(self,data):
        self.AR3_Feedback.joint_angles[2] = data.set_point + data.error
        self.AR3_Feedback.setpoint_angles[2] = data.set_point

    def joint4_callback(self,data):
        self.AR3_Feedback.joint_angles[3] = data.set_point + data.error
        self.AR3_Feedback.setpoint_angles[3] = data.set_point

    def joint5_callback(self,data):
        self.AR3_Feedback.joint_angles[4] = data.set_point + data.error
        self.AR3_Feedback.setpoint_angles[4] = data.set_point

    def joint6_callback(self,data):
        self.AR3_Feedback.joint_angles[5] = data.set_point + data.error
        self.AR3_Feedback.setpoint_angles[5] = data.set_point

    def send_joints(self):
        self.AR3FeedbackPub.publish(self.AR3_Feedback)

def main():
        rospy.init_node('Simulated_AR3_Feedback', anonymous='True')
        rate = rospy.Rate(60)

        feedback = SimulatedAR3Feedback()
        while not rospy.is_shutdown():
            feedback.send_joints()
            rate.sleep()
                

if __name__ == "__main__":
        main()
