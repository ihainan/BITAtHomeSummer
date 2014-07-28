#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : my_tf_broadcaster.py
# Author : csyls
# Created Date : 2014/07/21
# Descriptiont : tf坐标转换
import roslib
##roslib.load_manifest('learning_tf')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)    ##速率单位为赫兹，可修改
    while not rospy.is_shutdown():
        br.sendTransform((0.215, 0.0, 0.0),   ##转换坐标，x轴为21.5cm
                         tf.TransformBroadcaster.quaternion_from_euler(0.0, 0.0, 0.0),  ##旋转角度
                         rospy.Time.now(),   ##转换坐标时间差
                         "base_link",   ##车体
                         "base_laser")  ##Sick激光测距
        rate.sleep()