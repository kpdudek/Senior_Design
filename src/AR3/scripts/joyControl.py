#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from sensor_msgs.msg import Joy
from AR3.msg import AR3_Feedback
from Teensy.msg import AR3_Control
from RobotControllerClass import RobotController

def main():
    rospy.init_node('Joint_Control', anonymous='True')

    robot_controller = RobotControler()

    xbox = rospy.Subscriber('/joy', Joy, robot_controller.joy_callback)
    rate = rospy.Rate(60)

    robot_controller.set_controller_to_AR3()
    while not rospy.is_shutdown():
        robot_controller.stick_move()
        robot_controller.send_joints()
        rate.sleep()

if __name__ == "__main__":
	main()
