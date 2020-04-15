#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
import os
import sys
import getopt
from datetime import datetime

from Teensy.msg import AR3_Control
from AR3.msg import AR3_Feedback

def check_rob(data):
    global rob_there
    global control

    rob_there = True

    c = 0
    for ang in list(data.joint_angles):
        if ang <= 0.75*control.joint_angles[c] or ang >= 1.25*control.joint_angles[c]:
            rob_there = False
        c = c + 1


def main(argv):
    global rob_there
    global control

    rospy.init_node('Waypoint_Replay', anonymous='True')
    j_pub = rospy.Publisher('/AR3/Control', AR3_Control, queue_size=1)
    j_sub = rospy.Subscriber('/AR3/Feedback',AR3_Feedback,check_rob)
    control = AR3_Control()
    rate = rospy.Rate(30)

    try:
        opts, args = getopt.getopt(argv, "hf:")
    except getopt.GetoptError:
        print("No valid argument passed")
        sys.exit(2)

    # If no arguments are passed, print help
    if len(argv) == 0:
        print("No valid argument passed")
        sys.exit(2)

    # Take in the arguments
    for opt, arg in opts:
        if opt in ("-h"):
            print_info("./WaypointReplay.py -f /path/to/joint/angle/file.txt")
            sys.exit()
        elif opt in ("-f"):
            print("File loaded...")
            f = open(arg,'r')

    waypoint = 1
    for line in f:
        #Replay Feature
        print('Moving to waypoint: %d'%(waypoint))
        control.joint_angles = list(eval(line))

        j_pub.publish(control)
        rob_there = False

        while rob_there == False:
            j_pub.publish(control)
            rate.sleep()
            rate.sleep()
        waypoint += 1

    f.close()

if __name__ == "__main__":
    main(sys.argv[1:])
