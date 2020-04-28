#!/usr/bin/env python

import rospy
from math import sin
from math import cos
from math import atan2
import numpy as np
import os
import sys
import getopt
from datetime import datetime

from Teensy.msg import AR3_Control
from AR3.msg import AR3_Feedback

def edge_angle(ang1,ang2):
    # This function finds the signed shortest distance between two vectors
    vertex0 = [0.0,0.0]
    vertex1 = [cos(ang1),sin(ang1)]
    vertex2 = [cos(ang2),sin(ang2)]

    # Dot product of the vectors
    cosine_theta = vertex1[0]*vertex2[0] + vertex1[1]*vertex2[1]
    
    # Cross product of the vectors
    sin_theta = vertex1[0]*vertex2[1] - vertex1[1]*vertex2[0]
    
    # find the angle using the relationship tan(theta) = sin(theta)/cos(theta)
    edge_angle = atan2(sin_theta,cosine_theta)
    return edge_angle


def check_rob(data):
    global rob_there
    global control

    rob_there = True

    c = 0
    for ang in list(data.joint_angles):
        angle_diff = edge_angle(ang,control.joint_angles[c])
        if abs(angle_diff) > .25:
            rob_there = False
        c = c + 1


def main(argv):
    global rob_there
    global control

    rospy.init_node('Waypoint_Replay', anonymous='True')
    j_pub = rospy.Publisher('/AR3/Control', AR3_Control, queue_size=1)
    j_sub = rospy.Subscriber('/AR3/Feedback',AR3_Feedback,check_rob)
    control = AR3_Control()
    rate = rospy.Rate(7)
    control.run = 1

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

        # TODO: linearly interpolate between two configs and stream those angles to smooth the motion
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
