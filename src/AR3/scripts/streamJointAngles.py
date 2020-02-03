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
        eStop = data.eStop
        j1Angle = data.j1_current_angle

def main():
        global eStop
        global j1Angle
        eStop = 1
        j1Angle = 0.0
        rospy.init_node('Joint_Control', anonymous='True')

        j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
        j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
        j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
        j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)

        AR3 = rospy.Subscriber("/AR3/Debug", AR3_Debug, AR3_Callback)

        rate = rospy.Rate(30)

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()

        angles = [0,pi/2,pi,3*pi/2,2*pi]
        angleIdx = 0
        
        while not rospy.is_shutdown():
                if eStop == 0:
                        if ((abs(j1Angle - angles[angleIdx]) < .005) and (angleIdx < len(angles)-1)):
                                angleIdx = angleIdx + 1
                                # time.sleep(1)
                        
                        j1.data = angles[angleIdx]

                j1_pub.publish(j1)

                print('eStop: {} | Current Angle: {} | Setpoint: {}'.format(eStop,j1Angle,angles[angleIdx]))
                
                rate.sleep()
                
if __name__ == "__main__":
        main()
