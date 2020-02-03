#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
import sys

def main():
        rospy.init_node('Joint_Control', anonymous='True')

        j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)

        # rate = rospy.Rate(100)

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()

        angle = input("What angle to you want to go to (in radians): ")
        t_old = time.time()

        while not rospy.is_shutdown():
                j1.data = angle
                j1_pub.publish(j1)
                time.sleep(.5)
                

if __name__ == "__main__":
        main()
