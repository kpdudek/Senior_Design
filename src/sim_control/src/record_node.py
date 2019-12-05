#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from datetime import datetime

def getVal1(data):
    global J1, J1_count
    J1.data = data.data
    J1_count+=1
def getVal2(data):
    global J2, J2_count
    J2.data = data.data
    J2_count+=1
def getVal3(data):
    global J3, J3_count
    J3.data = data.data
    J3_count+=1
def getVal4(data):
    global J4, J4_count
    J4.data = data.data
    J4_count+=1

def main():
    rospy.init_node('Recorder', anonymous='True')

    global J1, J2, J3, J4, J1_count, J2_count, J3_count, J4_count

    J1 = Float64()
    J2 = Float64()
    J3 = Float64()
    J4 = Float64() 

    J1_count = 0
    J2_count = 0
    J3_count = 0
    J4_count = 0

    
    J1_sub = rospy.Subscriber('/rrbot/joint1_position_controller/command', Float64, getVal1)
    J2_sub = rospy.Subscriber('/rrbot/joint2_position_controller/command', Float64, getVal2)
    J3_sub = rospy.Subscriber('/rrbot/joint3_position_controller/command', Float64, getVal3)
    J4_sub = rospy.Subscriber('/rrbot/joint4_position_controller/command', Float64, getVal4)


    rate = rospy.Rate(60)

    #Record Feature 
    date = datetime.now()
    strDate = date.strftime("%m_%d_%y_%H:%M:%S")
    f = open("record_"+strDate+".txt",'w')
    timer = input("Input number of seconds to record for: ")
    val = 0
    while val != 60*timer:

        #Record Write
        f.write(str(J1.data)+","+str(J2.data)+","+str(J3.data)+","+str(J4.data)+"\n")
        val+=1
        rate.sleep()


    f.close()

if __name__ == "__main__":
    main()