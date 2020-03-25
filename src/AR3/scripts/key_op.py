#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
from AR3.msg import AR3_Debug
import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)
from RobotControllerClass import RobotController

class KeyPublisher(object):
    def __init__(self):
        self.key_val = String()
        self.key_pub = rospy.Publisher('/key_input',String,queue_size=1)
    
    def get_key(self):
        try:
            key = raw_input('Enter a pose/command: ')
            self.key_val.data = key

        except KeyboardInterrupt:
            self.key_val.data = 'home'
            self.send()
            rospy.signal_shutdown('Canceled by user')

    def send(self):
        self.key_pub.publish(self.key_val)

def main():
        rospy.init_node('Joint_Control', anonymous='True')

        key_op = KeyPublisher()
        while not rospy.is_shutdown():
            key_op.get_key()
            key_op.send()

                
if __name__ == "__main__":
        main()
