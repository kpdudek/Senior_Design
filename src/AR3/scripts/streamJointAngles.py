#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from AR3.msg import AR3_Debug

def AR3_Callback(data):
        global eStop
        global j1Angle
        global j2Angle
        global j3Angle
        global j4Angle
        global j5Angle
        global j6Angle

        eStop = data.eStop
        j1Angle = data.j1_current_angle
        j2Angle = data.j2_current_angle
        j3Angle = data.j3_current_angle
        j4Angle = data.j4_current_angle
        j5Angle = data.j5_current_angle
        j6Angle = data.j6_current_angle

def main():
        global eStop
        global j1Angle
        global j2Angle
        global j3Angle
        global j4Angle
        global j5Angle
        global j6Angle

        eStop = 1
        j1Angle = 0.0
        j2Angle = 0.0
        j3Angle = 0.0
        j4Angle = 0.0
        j5Angle = 0.0
        j6Angle = 0.0

        rospy.init_node('Joint_Control', anonymous='True')
        rate = rospy.Rate(30)

        j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)
        j5_pub = rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size = 1)
        j6_pub = rospy.Publisher('/rrbot/joint6_position_controller/command', Float64, queue_size = 1)

        AR3 = rospy.Subscriber("/AR3/Debug", AR3_Debug, AR3_Callback)

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        j5 = Float64()
        j6 = Float64()

        angles_j1 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j2 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j3 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j4 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j5 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j6 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]

        angleIdx = 0
        
        while not rospy.is_shutdown():
                if eStop == 0:
                        if ((abs(j2Angle - angles_j2[angleIdx]) < .005) and (abs(j3Angle - angles_j3[angleIdx]) < .005) and (angleIdx < len(angles_j2)-1)):
                                angleIdx = angleIdx + 1   
                        elif ((angleIdx == (len(angles_j2)-1)) and (abs(j2Angle - angles_j2[angleIdx]) < .005) and (abs(j3Angle - angles_j3[angleIdx]) < .005)):
                                angleIdx = 0
                        
                        j1.data = angles_j1[angleIdx]
                        j2.data = angles_j2[angleIdx]
                        j3.data = angles_j3[angleIdx]
                        j4.data = angles_j4[angleIdx]
                        j5.data = angles_j5[angleIdx]
                        j6.data = angles_j6[angleIdx]
                
                j1_pub.publish(j1)
                j2_pub.publish(j2)
                j3_pub.publish(j3)
                j4_pub.publish(j4)
                j5_pub.publish(j5)
                j6_pub.publish(j6)

                rate.sleep()
                
if __name__ == "__main__":
        main()
