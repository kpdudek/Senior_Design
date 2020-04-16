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

def main():
        rospy.init_node('Joint_Control', anonymous='True')
        rate = rospy.Rate(15)

        scale = 1.5
        step = 50
        angles_upper = np.linspace(0.0,scale,step)
        angles_lower = np.linspace((2.0*pi)-scale,2*pi,step)
        angles = list(angles_lower) + list(angles_upper) + list(reversed(angles_upper)) + list(reversed(angles_lower))

        robot_controller = RobotController()
        robot_controller.AR3Control.run = 1

        while not rospy.is_shutdown():
                for angle in angles:
                    robot_controller.AR3Control.joint_angles = [angle] * 6
                
                    robot_controller.send_joints()
                    rate.sleep()
                

if __name__ == "__main__":
        main()


