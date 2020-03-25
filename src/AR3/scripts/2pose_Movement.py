#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
from AR3.msg import AR3_Debug
import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController

class KeyListener(object):
    def __init__(self):
        self.key_val = ''
        self.last_pose = ''
        self.key_sub = rospy.Subscriber('/key_input',String,self.key_callback)

    def key_callback(self,msg):
        self.key_val = msg.data

def angle_check(set,current,tol):
        for idx in range(0,len(set)):
                if abs(set[idx]-current[idx]) > tol:
                        return False
        return True

def main():
        rospy.init_node('Joint_Control', anonymous='True')
        
        tpi = 2.0*pi
        pi2 = pi/2.0
        pi4 = pi/4.0
        z = 0.0

        home = [z,z,z,z,z,z]
        pose1 = [4.71,pi4,pi4,z,z,z]
        stage = [z,pi4,pi4,z,z,z]
        pose2 = [1.57,pi4,pi4,z,z,z]

        key_list = KeyListener()
        robot_controller = RobotController()

        rate = rospy.Rate(60)
        robot_controller.AR3Control.joint_angles = home
        robot_controller.send_joints()
        print('Starting at home...')

        robot_controller.AR3Control.run = 1
        while not rospy.is_shutdown():
                robot_controller.clc()

                if key_list.key_val == '1':
                    robot_controller.AR3Control.joint_angles = pose1
                    print('Move to: Pose 1')
                elif key_list.key_val == '2':
                    robot_controller.AR3Control.joint_angles = pose2
                    print('Move to: Pose 2')
                elif key_list.key_val == 's':
                    robot_controller.AR3Control.joint_angles = stage
                    print('Move to: Staging')
                elif key_list.key_val == 'home':
                    robot_controller.AR3Control.joint_angles = home
                    print('Move to: Home')
                elif key_list.key_val == 'close':
                    robot_controller.AR3Control.close_gripper = 1
                    print('Closing gripper...')
                elif key_list.key_val == 'open':
                    robot_controller.AR3Control.close_gripper = 0
                    print('Opening gripper...')
                else:
                    print('No matching action...')

                robot_controller.send_joints()
                rate.sleep()
                
if __name__ == "__main__":
        main()
