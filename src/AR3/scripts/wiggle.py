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

import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController

def main(argv):
        rospy.init_node('Joint_Control', anonymous='True')
        robot_controller = RobotController()
        rate = rospy.Rate(10)

        val = 0
        scale = 0.5
        step = 0.05
        while not rospy.is_shutdown():
                for idx in range(0,6):
                        robot_controller.AR3Control.joint_angles[idx] = (sin(val)*scale)
                robot_controller.send_joints()
                val = val + step
                rate.sleep()
                

if __name__ == "__main__":
        main(sys.argv[1:])


