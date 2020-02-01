#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64

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
	j3.data = 0
	j4.data = 0

	val = 0
	while not rospy.is_shutdown():
		j1_pub.publish(j1)
		j2_pub.publish(j2)
		j3_pub.publish(j3)
		j4_pub.publish(j4)

		rate.sleep()

if __name__ == "__main__":
	main()
