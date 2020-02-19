#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
import sys
import os,getopt
from joyControl import RobotController
from Teensy.msg import AR3_Control
from AR3.msg import AR3_Feedback


def main(argv):
        rospy.init_node('Joint_Control', anonymous='True')
        robot_controller = RobotController()
        rate = rospy.Rate(2)

        # Try to get the command line arguments
        try:
                opts, args = getopt.getopt(argv,"hr")

        except getopt.GetoptError:
                sys.exit(2)

                # If no arguments are passed, print help
        if len(argv) == 0:
                angle_1 = input("What angle to you want to move [J1] to (in radians): ")
                angle_2 = input("What angle to you want to move [J2] to (in radians): ")
                angle_3 = input("What angle to you want to move [J3] to (in radians): ")
                angle_4 = input("What angle to you want to move [J4] to (in radians): ")
                angle_5 = input("What angle to you want to move [J5] to (in radians): ")
                angle_6 = input("What angle to you want to move [J6] to (in radians): ")
                robot_controller.AR3Control.joint_angles = [angle_1,angle_2,angle_3,angle_4,angle_5,angle_6]
                
        else:
                # Take in the arguments
                for opt, arg in opts:
                        if opt in ("-h"):
                                robot_controller.AR3Control.joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]
                       
                        elif opt in ("-r"):
                                robot_controller.AR3Control.joint_angles = [0.0,0.8,1.8,0.0,5.53,0.0]


        while not rospy.is_shutdown():
                robot_controller.send_joints()
                rate.sleep()
                

if __name__ == "__main__":
        main(sys.argv[1:])
