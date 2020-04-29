#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from sensor_msgs.msg import Joy
from AR3.msg import AR3_Feedback
from Teensy.msg import AR3_Control

import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController
from JoyControlClass import JoyController

def main():
    rospy.init_node('Joy_Control', anonymous='True')

    robot_controller = JoyController()
    rate = rospy.Rate(60)

    robot_controller.set_controller_to_AR3()
    while not rospy.is_shutdown():
        if robot_controller.AR3Control.run == 1:   
            robot_controller.stick_move()
        robot_controller.send_joints()
        robot_controller.print_control()
        rate.sleep()

if __name__ == "__main__":
	main()
