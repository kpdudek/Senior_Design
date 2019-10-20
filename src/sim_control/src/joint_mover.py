#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64

def main():
	rospy.init_node('Joint_Control', anonymous='True')

	j_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size = 1)
	j2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size = 1)

	rate = rospy.Rate(30)

	x1 = Float64()
	x2 = Float64()

	val = 0
	while not rospy.is_shutdown():
		speed = .1
		amp = 1


		angle = sin(val)
		val += speed

		x1.data = angle * amp
		x2.data = -2*(angle * amp)

		j_pub.publish(x1)
		j2_pub.publish(x2)
		rate.sleep()

if __name__ == "__main__":
	main()
