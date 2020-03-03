#!/usr/bin/env python
from InterpreterNodeClass import InterpreterNode

def main():

  interNode = InterpreterNode()

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == "__main__":
  main()