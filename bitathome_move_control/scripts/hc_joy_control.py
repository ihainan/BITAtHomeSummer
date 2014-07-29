#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_joy_control.py
# Author : zhutianbao
# Created Date : 2014/7/28 15:10
# Description : 手柄控制

temp_speed = 300

import rospy
from bitathome_hardware_control.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


def joy_callback(data):
    global joyData
    joyData = data


def joy_loop():
    global joyData
    global pub, max_speed
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        if joyData is None or len(joyData.axes) == 0:
            continue
        elif joyData.axes[1] == 1:
            resp = ser(temp_speed, 0, 0)
            if resp.result == 1:
                print "前进 ：成功"
            else:
                print"前进 ：失败"
        elif joyData.axes[1] == -1:
            resp = ser(-temp_speed, 0, 0)
            if resp.result == 1:
                print "后退 ：成功"
            else:
                print"后退 ：失败"
        elif joyData.axes[0] == 1:
            resp = ser(0, 0, -300)
            if resp.result == 1:
                print "左转 ：成功"
            else:
                print"左转 ：失败"
        elif joyData.axes[0] == -1:
            resp = ser(0, 0, 300)
            if resp.result == 1:
                print "右转 ：成功"
            else:
                print"右转 ：失败"
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("hc_joy_control")

    ser = rospy.ServiceProxy("/hc_cmd_interface/vector_speed", VectorSpeed)
    joyData = Joy()
    pub = rospy.Subscriber("/joy", Joy, joy_callback)

    joy_loop()
