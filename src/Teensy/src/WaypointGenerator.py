#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
import time
from datetime import datetime

import sys
import os
import pwd
name = pwd.getpwuid( os.getuid() ).pw_name
file_path = '/home/%s/Senior_Design/src/AR3/scripts'%name
sys.path.insert(1,file_path)

def parse_state(state):
    state = state.split(',')
    angles = []

    for val in state:
        angle = val.strip(',')
        angles.append(float(angle))

    return str(angles)

def main():
    date = datetime.now()
    strDate = date.strftime("%m_%d_%y_%H:%M:%S")
    name = pwd.getpwuid( os.getuid() ).pw_name

    file_name = raw_input('Enter a file name (without extension) or hit enter to use the current time: ')

    if len(name) == 0:
        file_path = '/home/%s/Senior_Design/src/Teensy/records/record_%s.txt'%(name,strDate)
    else:
        file_path = '/home/%s/Senior_Design/src/Teensy/records/%s.txt'%(name,file_name)
    
    f = open(file_path,'w')
    print('Opened file with status: {}'.format(f))

    print('\nEnter robot state as comma separated values. All values will be cast to floats\nh --> Display help\nz --> All zeros\nend --> Save file')
    state = raw_input('\nEnter first joint state: ')

    while not state=='end':
        state = parse_state(state)
        f.write(state+"\n")
        state = raw_input('Enter the next state: ')

    try:
        f.close()
        print('Text file closed...')
    except:
        print('Couldnt close text file...')

        

                
            

if __name__ == "__main__":
    main()