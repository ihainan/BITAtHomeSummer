#!/usr/bin/env python
# encoding:utf-8
# Filename : my_tf_broadcaster.py
# Author : csyls
# Created Date : 2014/07/21
# Descriptiont : tf坐标转换
import roslib
##roslib.load_manifest('learning_tf')
import rospy
import tf

node_name = "my_tf_broadcaster"

if __name__ == '__main__':
    rospy.init_node(node_name)
    r = rospy.Rate(50)    ##速率单位为赫兹，可修改
    br = tf.TransformBroadcaster()
    while True:
        br.sendTransform((-0.215, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "base_link", "laser")
        r.sleep()

    rospy.spin()
