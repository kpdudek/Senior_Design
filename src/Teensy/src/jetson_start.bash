#!/bin/bash

cd "~/Senior_Design"
catkin build
source devel/setup.bash
roslaunch Teensy jetson_launch_v1.launch