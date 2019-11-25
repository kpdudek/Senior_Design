#!/usr/bin/env python

import rospy
from math import sin
from math import pi
import numpy as np
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
# import trac_ik
from trac_ik_python.trac_ik import IK
#from trac_ik_python.trac_ik_wrap import TRAC_IK



def main():

    rospy.init_node('Joint_Control', anonymous='True')

    j1_pub = rospy.Publisher(
        '/rrbot/joint1_position_controller/command', Float64, queue_size=1)
    j2_pub = rospy.Publisher(
        '/rrbot/joint2_position_controller/command', Float64, queue_size=1)
    j3_pub = rospy.Publisher(
        '/rrbot/joint3_position_controller/command', Float64, queue_size=1)
    j4_pub = rospy.Publisher(
        '/rrbot/joint4_position_controller/command', Float64, queue_size=1)

    rate = rospy.Rate(60)

    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()

    with open('/home/kurt/Senior_Design/src/sim_control/src/irac.urdf', 'r') as file:
        urdf = file.read()
    Solver = IK("world", "flange", urdf_string=urdf)

    # print("IK solver uses link chain:")
    # print(Solver.link_names)

    # print("IK solver base frame:")
    # print(Solver.base_link)

    # print("IK solver tip link:")
    # print(Solver.tip_link)

    # print("IK solver for joints:")
    # print(Solver.joint_names)

    # print("IK solver using joint limits:")
    # lb, up = Solver.get_joint_limits()
    # print("Lower bound: " + str(lb))
    # print("Upper bound: " + str(up))
    

    qinit = [0.0] * Solver.number_of_joints
    x = .5; y = 0.0 ; z = 0.4
    rx = 0; ry = .707; rz = 0.0
    rw = .707
    
    quat = [rx,ry,rz,rw]
    eulerAng = euler_from_quaternion(quat)
    print(eulerAng)

    rand_coords = [(.5,0.0,.4),(0.4,0.0,.4),(0.0,0.0,.5)]
    NUM_COORDS = 1
    avg_time = 0.0
    num_solutions_found = 0
    bx = by = bz = .03
    brx = bry = brz = 2*pi * .1
    sol = []
    for x, y, z in rand_coords:
        ini_t = time.time()
        sol.append(Solver.get_ik(qinit,
                               x, y, z,
                               rx, ry, rz, rw,
                               bx, by, bz,
                               brx, bry, brz))
        qinit = [0.0] * Solver.number_of_joints
        fin_t = time.time()
        call_time = fin_t - ini_t
        print "IK call took: " + str(call_time)
        avg_time += call_time
        if sol:
            # print "X, Y, Z: " + str( (x, y, z) )
            # print "SOL: " + str(sol)
            num_solutions_found += 1

    avg_time = avg_time / NUM_COORDS

    print
    # print "Found " + str(num_solutions_found) + " of 200 random coords ignoring everything"
    print "Average IK call time: " + str(avg_time)
    print(sol)

    userIn = raw_input("Can the robot begin moving? Ensure the space is clear (y/n): ")
    if userIn != "y":
        raise ValueError('Robot wont begin moving.')

    idx = 0
    while not rospy.is_shutdown():
        angs = sol[idx]
        j1.data = angs[0]
        j2.data = angs[1]
        j3.data = angs[2]
        j4.data = angs[3]

        j1_pub.publish(j1)
        j2_pub.publish(j2)
        j3_pub.publish(j3)
        j4_pub.publish(j4)

        time.sleep(3)
        if idx < len(sol)-1:
            idx +=1

        rate.sleep()


if __name__ == "__main__":
    main()
