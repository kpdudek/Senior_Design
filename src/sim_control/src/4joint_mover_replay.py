#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64

from datetime import datetime

def main():
	rospy.init_node('Joint_Control', anonymous='True')

	j1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
	j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)
	j3_pub = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size = 1)
	J4_pub = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size = 1)


	rate = rospy.Rate(30)

	J4 = Float64()
	j1 = Float64()
	j2 = Float64()
	j3 = Float64()

	#Replay Feature 
	strDate = raw_input("Enter record as 'm_d_y_H:M:S: ")
	f = open("record_"+strDate+".txt",'r')


	for line in f:

		#Replay Feature
		jointAngs = list(eval(line))
		j1.data = jointAngs[0]
		j2.data = jointAngs[1]
		j3.data = jointAngs[2]
		J4.data = jointAngs[3]

		j1_pub.publish(j1)
		j2_pub.publish(j2)
		j3_pub.publish(j3)
		J4_pub.publish(J4)
		
		rate.sleep()

	f.close()

if __name__ == "__main__":
	main()
