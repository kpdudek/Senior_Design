#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from AR3.msg import AR3_Debug
    

def main():
    rospy.init_node('Joint_Control', anonymous='True')

    while not rospy.is_shutdown():
        t = time.time()
        # rospy.signal_shutdown()




if __name__ == "__main__":
	main()
