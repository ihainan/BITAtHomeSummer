#!/usr/bin/env python2.7
# coding:utf-8
# Filename : add_two_ints_server.py
# Author : ihainan
# E-mail : ihainan@bitathome.org
# Created Date : 2014/07/22 13:28
# History
#   2014/07/22 13:28 ： 修复无法修改速度的 bug [曹大帅]

from beginner_tutorials.srv import *
import rospy


def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
