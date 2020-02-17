#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from AR3.msg import AR3_Debug


def clc():
    print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
    print('AR3@Controller:$ '),

class Joints(object):
    def __init__(self):
        # ROS stuff
        self.joint1 = Float64() 
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joint5 = Float64()
        self.joint6 = Float64()

        self.AR3_joint1 = 0.0
        self.AR3_joint2 = 0.0
        self.AR3_joint3 = 0.0
        self.AR3_joint4 = 0.0
        self.AR3_joint5 = 0.0
        self.AR3_joint6 = 0.0

        self.j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        self.j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        self.j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        self.j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)
        self.j5_pub = rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size = 1)
        self.j6_pub = rospy.Publisher('/rrbot/joint6_position_controller/command', Float64, queue_size = 1)

        self.AR3_Sub = rospy.Subscriber('/AR3/Debug', AR3_Debug, self.AR3_Callback)

        # Motion parameters
        self.joint_idx = 0
        self.joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.ka = .005

        # Joystick axes and buttons
        self.left_stick_val = 0
        self.left_stick_press = 0
        self.B = 0
        self.A = 0
        self.Y = 0

        t = time.time()
        t_old = time.time()
        
        while (t-t_old) < 4:
            self.set_controller_to_AR3()
            t = time.time()
        
        self.print_control()

    def AR3_Callback(self,data):
        self.AR3_joint1 = data.j1_current_angle
        self.AR3_joint2 = data.j2_current_angle
        self.AR3_joint3 = data.j3_current_angle
        self.AR3_joint4 = data.j4_current_angle
        self.AR3_joint5 = data.j5_current_angle
        self.AR3_joint6 = data.j6_current_angle
    
    def set_controller_to_AR3(self):
        self.joint_angles[0] = self.AR3_joint1
        self.joint_angles[1] = self.AR3_joint2
        self.joint_angles[2] = self.AR3_joint3
        self.joint_angles[3] = self.AR3_joint4
        self.joint_angles[4] = self.AR3_joint5
        self.joint_angles[5] = self.AR3_joint6
    
    def home(self):
        self.joint1.data = 0.0
        self.joint2.data = 0.0
        self.joint3.data = 0.0
        self.joint4.data = 0.0
        self.joint5.data = 0.0
        self.joint6.data = 0.0

        self.joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]

    def joy_callback(self,data):
        self.A = data.buttons[0]
        self.B = data.buttons[1]
        self.Y = data.buttons[3]
        self.left_stick_val = -1.0 * data.axes[0]
        self.left_stick_press = data.buttons[9]

        # Increment the joint being moved based upon left stick press
        if self.A == 1:
            self.joint_idx += 1
            if self.joint_idx > 5:
                self.joint_idx = 0
            self.print_control()
        elif self.B == 1:
            self.joint_idx -= 1
            if self.joint_idx < 0:
                self.joint_idx = 5
            self.print_control()

        if self.Y == 1:
            self.home()


    def stick_move(self):
        # Modify one joint angle based upon
        self.joint_angles[self.joint_idx] = self.joint_angles[self.joint_idx] + (self.left_stick_val*self.ka)

        # Convert to global angle
        if self.joint_angles[self.joint_idx] < 0:
            self.joint_angles[self.joint_idx] = (2*pi) + self.joint_angles[self.joint_idx]
        elif self.joint_angles[self.joint_idx] >= 2*pi:
            self.joint_angles[self.joint_idx] = self.joint_angles[self.joint_idx] - 2*pi
    
    def send_joints(self):
        self.joint1.data = self.joint_angles[0]
        self.joint2.data = self.joint_angles[1]
        self.joint3.data = self.joint_angles[2]
        self.joint4.data = self.joint_angles[3]
        self.joint5.data = self.joint_angles[4]
        self.joint6.data = self.joint_angles[5]

        self.j1_pub.publish(self.joint1)
        self.j2_pub.publish(self.joint2)
        self.j3_pub.publish(self.joint3)
        self.j4_pub.publish(self.joint4)
        self.j5_pub.publish(self.joint5)
        self.j6_pub.publish(self.joint6)
    
    def print_control(self):
        clc()
        print('Moving joint: [{}]'.format(self.joint_idx+1))

def main():
    rospy.init_node('Joint_Control', anonymous='True')

    joints = Joints()

    xbox = rospy.Subscriber('/joy', Joy, joints.joy_callback)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        joints.stick_move()
        joints.send_joints()
        rate.sleep()

if __name__ == "__main__":
	main()
