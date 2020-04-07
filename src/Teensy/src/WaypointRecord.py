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

def angles(data):
    global curr_angles = data.joint_angles 

def main():
    global curr_angles

    rospy.init_node('Recorder', anonymous='True')

    robot_controller = JoyController()

    xbox = rospy.Subscriber('/joy', Joy, robot_controller.joy_callback)
    angles = rospy.Subscriber('/AR3/Control',AR3_Control,angles)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():   
        
        if robot_controller.A == 1:
            #Record Feature 
            date = datetime.now()
            strDate = date.strftime("%m_%d_%y_%H:%M:%S")
            f = open("record_"+strDate+".txt",'w')
    
            while robot_controller.A == 1:
                if robot_controller.B == 1:
                    #Record Write
                    f.write(str(curr_angles)+"\n")
                rate.sleep()
            f.close()
        rate.sleep()

if __name__ == "__main__":
    main()