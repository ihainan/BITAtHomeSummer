#!/usr/bin/env python
#-*-encoding:utf-8-*-
#AUTHOR:jinzeshang
#NAME:hc_keyboard_control.py
#CREAT DATE:2014/7/28 10:01

import roslib
import rospy
import threading
import os
import sys
import math
import tty, termios
from bitathome_hardware_control.srv import *

default_speed = 400
half_default_speed = 200


def keyboard_loop():
    global ser, default_speed
    rate = rospy.Rate(2)

    print "使用WASD控制行走，使用JK控制旋转，按E停止运动，按Q退出控制"

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    while not rospy.is_shutdown():
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == "w":
            resp = ser(default_speed, -default_speed, 0)
        elif ch == "s":
            resp = ser(-default_speed, default_speed, 0)
        elif ch == "a":
            resp = ser(-half_default_speed, -half_default_speed, default_speed)
        elif ch == "d":
            resp = ser(half_default_speed, half_default_speed, -default_speed)
        elif ch == "j":
            resp = ser(-default_speed, -default_speed, -default_speed)
        elif ch == "k":
            resp = ser(default_speed, default_speed, default_speed)
        elif ch == "e":
            resp = ser(0, 0, 0)
        elif ch == "q":
            exit()
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("hc_keyboard_control")

    ser = rospy.ServiceProxy("/hc_cmd_interface/motor_speed", MotorSpeed)

    keyboard_loop()


