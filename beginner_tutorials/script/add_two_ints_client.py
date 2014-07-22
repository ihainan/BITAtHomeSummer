#!/usr/bin/env python2.7
# coding:utf-8
# Filename : add_two_ints_client.py
# Author : ihainan
# E-mail : ihainan@bitathome.org
# Created Date : 2014/07/22 13:28
# History
#   2014/07/22 13:28 ： 修复无法修改速度的 bug [曹大帅]

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
