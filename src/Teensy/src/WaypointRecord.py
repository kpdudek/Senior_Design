#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from sensor_msgs.msg import Joy
from AR3.msg import AR3_Feedback
from Teensy.msg import AR3_Control
from datetime import datetime

import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass_Ethan import RobotController
from JoyControlClass_Ethan import JoyController

def angles(data):
    global curr_angles
    curr_angles = data.joint_angles

def store_curr(data):
    global curr_angles
    global robot_controller
    global counter
    global f

    robot_controller.joy_callback(data)

    if robot_controller.A == 1:
            counter = counter + 1

    if robot_controller.A == 1 and counter%2 == 0:
        #Record Feature 
        date = datetime.now()
        strDate = date.strftime("%m_%d_%y_%H:%M:%S")
        name = pwd.getpwuid( os.getuid() ).pw_name
        file_path = '/home/%s/Senior_Design/src/Teensy/records/record_%s.txt'%(name,strDate)
        f = open(file_path,'w')
        print('Opened file with status: {}'.format(f))
    elif robot_controller.A == 1 and counter%2 != 0:
        try:
            f.close()
            print('Text file closed...')
        except:
            print('Couldnt close text file...')


    if robot_controller.B == 1 and counter%2 == 0:
        #Record Write
        try:
            f.write(str(curr_angles)+"\n")
            print('Waypoint saved...')
        except:
            print('No control values exist...')
    elif robot_controller.B == 1:
        print('Open a text file first (press A)...')
        


def main():
    global curr_angles
    global robot_controller
    global counter
    global f

    rospy.init_node('Recorder', anonymous='True')

    robot_controller = JoyController()

    j_angles = rospy.Subscriber('/AR3/Control',AR3_Control,angles)
    xbox = rospy.Subscriber('/joy', Joy, store_curr)
    rate = rospy.Rate(30)

    counter = 1
    counter_prev = 1

    while not rospy.is_shutdown():
        rate.sleep()
        

                
            

if __name__ == "__main__":
    main()