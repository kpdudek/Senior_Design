#!/usr/bin/env python

import rospy
from math import sin
import numpy as np
from std_msgs.msg import Float64
import os
import sys
import getopt
from datetime import datetime


def main(argv):
	rospy.init_node('Joint_Control', anonymous='True')

	j1_pub = rospy.Publisher(
	    '/rrbot/joint1_position_controller/command', Float64, queue_size=1)
	j2_pub = rospy.Publisher(
	    '/rrbot/joint2_position_controller/command', Float64, queue_size=1)
	j3_pub = rospy.Publisher(
	    '/rrbot/joint3_position_controller/command', Float64, queue_size=1)
	J4_pub = rospy.Publisher(
	    '/rrbot/joint4_position_controller/command', Float64, queue_size=1)

	rate = rospy.Rate(30)

	J4 = Float64()
	j1 = Float64()
	j2 = Float64()
	j3 = Float64()

	try:
		opts, args = getopt.getopt(argv, "hf:")

	except getopt.GetoptError:
		print("No valid argument passed")
		sys.exit(2)

	# If no arguments are passed, print help
	if len(argv) == 0:
		print("No valid argument passed")
		sys.exit(2)

	# Take in the arguments
	for opt, arg in opts:
		if opt in ("-h"):
			print_info("./4joint_mover_replay.py -f /path/to/joint/angle/file.txt")
			sys.exit()
		elif opt in ("-f"):
			print("File loaded...")
			f = open(arg,'r')

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
	main(sys.argv[1:])
