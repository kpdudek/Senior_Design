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

	#Record Feature 
	date = datetime.now()
	strDate = date.strftime("%m_%d_%y_%H:%M:%S")
	f = open("record_"+strDate+".txt",'w')


	val = 0
	while val < 10:
		speed = .05
		amp = 1


		angle = sin(val)
		val += speed

		J4.data = (angle * 2*amp)
		j1.data = (angle * .2*amp)
		j2.data = -(angle * .5*amp)
		j3.data = (angle * 2*amp)

		#Record Write
		f.write(str(j1.data)+","+str(j2.data)+","+str(j3.data)+","+str(J4.data)+"\n")

		j1_pub.publish(j1)
		j2_pub.publish(j2)
		j3_pub.publish(j3)
		J4_pub.publish(J4)
		
		rate.sleep()

	f.close()

if __name__ == "__main__":
	main()
