#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
import sys
import os,getopt


def main(argv):
        rospy.init_node('Joint_Control', anonymous='True')

        j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)
        j5_pub = rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size = 1)
        j6_pub = rospy.Publisher('/rrbot/joint6_position_controller/command', Float64, queue_size = 1)

        rate = rospy.Rate(2)

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()

        # Try to get the command line arguments
        try:
                opts, args = getopt.getopt(argv,"hr")

        except getopt.GetoptError:
                print_info()
                sys.exit(2)

                # If no arguments are passed, print help
        if len(argv) == 0:
                angle_1 = input("What angle to you want to move [J1] to (in radians): ")
                angle_2 = input("What angle to you want to move [J2] to (in radians): ")
                angle_3 = input("What angle to you want to move [J3] to (in radians): ")
                angle_4 = input("What angle to you want to move [J4] to (in radians): ")
                angle_5 = input("What angle to you want to move [J5] to (in radians): ")
                angle_6 = input("What angle to you want to move [J6] to (in radians): ")
                
                if angle_6 < 0.0:
                        angle_6 = (2*pi)+angle_6
                if angle_5 < 0.0:
                        angle_5 = (2*pi)+angle_5
                if angle_4 < 0.0:
                        angle_4 = (2*pi)+angle_4
                if angle_3 < 0.0:
                        angle_3 = (2*pi)+angle_3
                if angle_2 < 0.0:
                        angle_2 = (2*pi)+angle_2
                if angle_1 < 0.0:
                        angle_1 = (2*pi)+angle_1
        else:
                # Take in the arguments
                for opt, arg in opts:
                        if opt in ("-h"):
                                angle_1 = 0.0
                                angle_2 = 0.0
                                angle_3 = 0.0
                                angle_4 = 0.0
                                angle_5 = 0.0
                                angle_6 = 0.0
                        elif opt in ("-r"):
                                angle_1 = 0.0
                                angle_2 = 0.8
                                angle_3 = 1.57
                                angle_4 = 0.0
                                angle_5 = 5.53
                                angle_6 = 0.0

        while not rospy.is_shutdown():
                j1.data = angle_1
                j1_pub.publish(j1)

                j2.data = angle_2
                j2_pub.publish(j2)

                j3.data = angle_3
                j3_pub.publish(j3)

                j4.data = angle_4
                j4_pub.publish(j4)

                j5.data = angle_5
                j5_pub.publish(j5)

                j6.data = angle_6
                j6_pub.publish(j6)

                rate.sleep()
                

if __name__ == "__main__":
        main(sys.argv[1:])
