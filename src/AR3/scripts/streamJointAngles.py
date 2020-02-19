#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from AR3.msg import AR3_Debug
import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController

def angle_check(set,current,tol):
        for idx in range(0,len(set)):
                if abs(set[idx]-current[idx]) > tol:
                        return False
        return True

def main():
        rospy.init_node('Joint_Control', anonymous='True')
        rate = rospy.Rate(60)

        robot_controller = RobotController()

        # angles_j1 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        # angles_j2 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        # angles_j3 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        # angles_j4 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        # angles_j5 = [0.0, pi/3.0, 0.0, 5.0*pi/3.0]
        # angles_j6 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]

        angles_j1 = [0.0, pi/2.0]
        angles_j2 = [0.0, pi/6.0]
        angles_j3 = [0.0, pi/2.0]
        angles_j4 = [0.0, pi/2.0]
        angles_j5 = [0.0, pi/3.0]
        angles_j6 = [0.0, pi/2.0]

        angleIdx = 0
        set_angles = [angles_j1[angleIdx],angles_j2[angleIdx],angles_j3[angleIdx],angles_j4[angleIdx],angles_j5[angleIdx],angles_j6[angleIdx]]
        robot_controller.AR3Control.joint_angles = set_angles
        while not rospy.is_shutdown():
                robot_controller.AR3Feedback.joint_angles = list(robot_controller.AR3Feedback.joint_angles)

                if angle_check(set_angles,robot_controller.AR3Feedback.joint_angles,0.0005) and (angleIdx+1 < len(angles_j2)):
                        angleIdx = angleIdx + 1   
                elif ((angleIdx == (len(angles_j2)-1)) and angle_check(set_angles,robot_controller.AR3Feedback.joint_angles,0.0005)):
                        angleIdx = 0
                
                set_angles = [angles_j1[angleIdx],angles_j2[angleIdx],angles_j3[angleIdx],angles_j4[angleIdx],angles_j5[angleIdx],angles_j6[angleIdx]]
                robot_controller.AR3Control.joint_angles = set_angles
                robot_controller.send_joints()
                rate.sleep()
                
if __name__ == "__main__":
        main()
