#!/usr/bin/env python2.7
# coding:utf-8
# Filename : talker.py
# Author : ihainan
# E-mail : ihainan@bitathome.org
# Created Date : 2014/07/22 10:30
# History
#   2014/07/22 10:32 ： 修改了 Topic 类型 [曹大帅]


import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
