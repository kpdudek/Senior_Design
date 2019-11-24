#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import threading

class GetUserInput(threading.Thread):
    def __init__(self,move):
        threading.Thread.__init__(self)
        # self.move = False
        # thread = threading.Thread(target=self.run, args=())
        # thread.daemon = True                            # Daemonize thread
        # thread.start() 

    def run(self):
        global move
        while True:
            userIn = raw_input("Start moving the robot (y/n): ")
            if userIn == "y":
                move = True
            elif userIn == "n":
                move = False
            else:
                print("[W]  Unrecognized input")


def main():
    rospy.init_node('Joint_Control', anonymous='True')

    j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
    j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
    j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
    j4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)


    rate = rospy.Rate(60)

    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()

    j1.data = 0
    j2.data = 0
    j3.data = -pi/6
    j4.data = -pi/6
    val = pi

    global move
    move = False
    inputThread = GetUserInput(move)
    inputThread.start() 

    while not rospy.is_shutdown():
        deltaT = .05
        amp = .25

        # print(val)
        # print(move)
        if move: 
            j1.data = sin(val) * amp * pi
            val += deltaT

        j1_pub.publish(j1)
        j2_pub.publish(j2)
        j3_pub.publish(j3)
        j4_pub.publish(j4)
            
        rate.sleep()

if __name__ == "__main__":
    main()
