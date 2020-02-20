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

class Interpreter(object):
    def __init__(self):
        self.joint1 = Float64()
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joint5 = Float64()
        self.joint6 = Float64()
        
        self.j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        self.j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        self.j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        self.j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)
        self.j5_pub = rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size = 1)
        self.j6_pub = rospy.Publisher('/rrbot/joint6_position_controller/command', Float64, queue_size = 1)

        self.AR3ControlSub = rospy.Subscriber('/AR3/Control', AR3_Control, self.AR3ControlCallback)
    
    def AR3ControlCallback(self,data):
        data.joint_angles = list(data.joint_angles)
        
        self.joint1.data = data.joint_angles[0]
        self.joint2.data = data.joint_angles[1]
        self.joint3.data = data.joint_angles[2]
        self.joint4.data = data.joint_angles[3]
        self.joint5.data = data.joint_angles[4]
        self.joint6.data = data.joint_angles[5]

    def send_joints(self):
        self.j1_pub.publish(self.joint1.data)
        self.j2_pub.publish(self.joint2.data)
        self.j3_pub.publish(self.joint3.data)
        self.j4_pub.publish(self.joint4.data)
        self.j5_pub.publish(self.joint5.data)
        self.j6_pub.publish(self.joint6.data)

def main():
        rospy.init_node('Interpreter', anonymous='True')
        rate = rospy.Rate(60)

        interpreter = Interpreter()
        while not rospy.is_shutdown():
            interpreter.send_joints()
            rate.sleep()
                

if __name__ == "__main__":
        main()
