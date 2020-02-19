#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from AR3.msg import AR3_Debug
from RobotControllerClass import RobotController

def main():
        rospy.init_node('Joint_Control', anonymous='True')
        rate = rospy.Rate(60)

        robot_controller = RobotController()

        angles_j1 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        angles_j2 = [0.0, pi/6.0, 0.0, 11.0*pi/6.0]
        angles_j3 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        angles_j4 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]
        angles_j5 = [0.0, pi/3.0, 0.0, 5.0*pi/3.0]
        angles_j6 = [0.0, pi/2.0, 0.0, 3.0*pi/2.0]


        
        angleIdx = 0
        while not rospy.is_shutdown():
                if robot_controller.eStop == 0:
                        if ((abs(j2Angle - angles_j2[angleIdx]) < .005) and (abs(j3Angle - angles_j3[angleIdx]) < .005) and (angleIdx < len(angles_j2)-1)):
                                angleIdx = angleIdx + 1   
                        elif ((angleIdx == (len(angles_j2)-1)) and (abs(j2Angle - angles_j2[angleIdx]) < .005) and (abs(j3Angle - angles_j3[angleIdx]) < .005)):
                                angleIdx = 0
                        
                        robot
                
                j1_pub.publish(j1)
                j2_pub.publish(j2)
                j3_pub.publish(j3)
                j4_pub.publish(j4)
                j5_pub.publish(j5)
                j6_pub.publish(j6)

                rate.sleep()
                
if __name__ == "__main__":
        main()
