#!/usr/bin/env python3

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp = add_two_ints(x, y)
        return resp.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sub_two_ints_client(x, y):
    rospy.wait_for_service('sub_two_ints')
    try:
        sub_two_ints = rospy.ServiceProxy('sub_two_ints', SubTwoInts)
        resp = sub_two_ints(x, y)
        return resp.sub
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def mul_two_ints_client(x, y):
    rospy.wait_for_service('mul_two_ints')
    try:
        mul_two_ints = rospy.ServiceProxy('mul_two_ints', MulTwoInts)
        resp = mul_two_ints(x, y)
        return resp.mul
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def div_two_ints_client(x,y):
    rospy.wait_for_service('div_two_ints')
    try:
        div_two_ints=rospy.ServiceProxy('div_two_ints',DivTwoInts)
        resp=div_two_ints(x,y)
        return resp.div
    except rospy.ServiceException as e:
        print("Service call failed %s"%e)

def usage():
    return "%s [A <operator> y]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 4:
        x,y = int(sys.argv[1]),int(sys.argv[3])
        if sys.argv[2]=="+":
            print("Requesting %s+%s"%(x, y))
            print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))

        elif sys.argv[2]=="-":
            print("Requesting %s-%s"%(x, y))
            print("%s - %s = %s"%(x, y, sub_two_ints_client(x, y)))

        elif sys.argv[2]=="x":
            print("Requesting %sx%s"%(x, y))
            print("%s x %s = %s"%(x, y, mul_two_ints_client(x, y)))
        # In most shells, including the default Bash shell, the * character is a special character used for 
        # pattern matching or globbing. As a result, when you pass * as an argument in the command line, 
        # the shell performs expansion on it before your Python script receives the arguments. This means 
        # that the * character gets replaced with a list of files in the current directory.

        elif sys.argv[2]=="/":
            print("Requesting %s/%s"%(x, y))
            print("%s / %s = %s"%(x, y, div_two_ints_client(x, y)))

    else:
        print(usage())
        sys.exit(1)
   
    
