#!/usr/bin/env python
# encoding:utf-8
# Filename : hc_avoid_obstacle.py
# Author : Majunbang
# Created Date : 2014/7/30 10:17
# Description : 避障算法

import rospy
import time
import threading
from math import *
from sensor_msgs.msg import *
from bitathome_vision_control.msg import *
from bitathome_hardware_control.srv import *

data_sick = []
data_laser = []
th_target = 100
x_target = 0
y_target = 0
x = 0
y = 0
threshold = [1]  # 不同的阈值
base_r = 0.21 / cos(pi / 6)  # 底座半径
trans = 0.00023  # 轮子转速为１的时候实际运动速度为0.00023


#求可行方向
def find_dir(dis, r):
    sectors = []  # 存放可用的扇区
    sec = []  # 备份扇区
    flag = 0  # 用于标记是不是首个符合要求的数据
    # 找到可用的扇区并把相邻的扇区合并
    tmp = []
    for i in range(0, len(dis)):
        if dis[i] > r:
            if flag is 0:
                if i is len(dis) - 1:
                    sectors.append([i, i])
                    sec.append([i, i])
                else:
                    tmp.append(i)
                    flag = 1
            else:
                if i is len(dis) - 1:
                    tmp.append(i)
                    sectors.append(tmp)
                    sec.append(tmp)
        else:
            if flag is not 0:
                tmp.append(i - 1)
                sectors.append(tmp)
                sec.append(tmp)
                tmp = []
                flag = 0
    if len(sectors) is 0:
        return sectors
    #以扇形区角平分线为可行方向
    for i in range(0, len(sectors)):
        th = ((sectors[i][0] + sectors[i][1] + 1) * 3 + 60 - 180) / 2 * pi / 180
        sectors[i].append(th)
    #验证是否满足条件
    k = 0
    for i in range(0, len(sec)):
        #弦长条件
        if 0.24 * 1.2 > r * sin((sec[i][1] - sec[i][0] + 1) * 3 * pi / 180 / 2):
            del sectors[i - k]
            k += 1
            continue
        #矩形条件
        d = r * cos((sec[i][1] - sec[i][0] + 1) * 3 * pi / 180 / 2)  # 矩形的长
        #角度范围
        th1 = (sec[i][0] * 3 + 30) * pi / 180 - pi / 2
        th2 = ((sec[i][1] + 1) * 3 + 30) * pi / 180 - pi / 2
        if sec[i][2] <= 0:
            for j in range(0, len(dis)):
                th = (j * 3 + 30 - 90) * pi / 180  # 第ｊ个点的角度
                if th > sec[i][2] + pi / 2:
                    break
                elif th < 0 and (th < th1 or th > th2):
                    if dis[j] < 0.24 * 1.2 / sin(sec[i][2] - th):  # 有点落在矩形内
                        del sectors[i - k]
                        k += 1
                        break
                elif th > 0 and (th < th1 or th > th2):
                    if dis[j] < 0.24 * 1.2 / cos(sec[i][2] + pi / 2 - th):
                        del sectors[i - k]
                        k += 1
                        break
        else:
            for j in range(0, len(dis)):
                th = (j * 3 + 30 - 90) * pi / 180  # 第ｊ个点的角度
                if th < sec[i][2] - pi / 2:
                    continue
                elif th < 0 and (th < th1 or th > th2):
                    if dis[j] < 0.24 * 1.2 / sin(sec[i][2] - th):  # 有点落在矩形内
                        del sectors[i - k]
                        k += 1
                        break
                elif th > 0 and (th < th1 or th > th2):
                    if dis[j] < 0.24 * 1.2 / cos(sec[i][2] + pi / 2 - th):
                        del sectors[i - k]
                        k += 1
                        break
    return sectors


#处理数据

def manage_data(data_laser):
    global threshold
    distance = []
    #取-90~90度，将它分为60组，每组六个数据
    for i in range(151, 391, 6):
        distance.append(data_laser[i: i+6])
    #取每组中的最小值作为这组的标记值并存放到dis中
    dis = []
    for i in distance:
        tmp = 3
        for j in i:
            if (j < tmp) and (j > 0.009):
                tmp = j
        dis.append(tmp)
    sec = []  # 用于存放最大的阈值得到的可行方向
    R = 0
    rospy.loginfo("阈值")
    print threshold
    sec = find_dir(dis, 1)
    print sec
    if len(sec) is 0:
        rospy.loginfo("没找着可行方向")
        ser(0, 0, 400)
        time.sleep(2)
        return 0
    else:
        rospy.loginfo("可行区间")
        print sec
        th_best = sec[0][2]
        for i in sec:
            if abs(i[2]) < abs(th_best):
                th_best = i[2]
        th_best = -th_best
        print "旋转角度:%d" %(th_best/pi * 180)
        print (th_best / 2.5 / trans)
        if abs(th_best / 2.5 / trans) > 2000:
            ser(0, 0, 2000)
            time.sleep(2)
            ser(2000, 0, 0)
            time.sleep(2.5)
            return 1
        else:
            print "准备旋转"
            if abs(th_best) < 10 * pi / 180:  # 低于10度不转
                print "前进"
                ser(2000, 0, 0)
                time.sleep(2.5)
                return 1
            else:
                print "转弯"
                ser(0, 0, th_best / 2 / trans)
                time.sleep(2)
                ser(2000, 0, 0)
                time.sleep(2.5)
                ser(0, 0, -th_best / 2 / trans)
                time.sleep(2)
                return 1



#读取数据
def read_data():
    rospy.Subscriber("/scan", LaserScan, avoid_obstacle)
    rospy.spin()


#读取kinect
def read_kinect():
    rospy.Subscriber("/KinectVision/direction", Direction, change_direction)
    rospy.spin()


#将数据存到data_laser
def avoid_obstacle(data):
    global data_sick
    data_sick = data.ranges


#读取坐标转化为角度
def change_direction(data):
    global x, y
    x = data.x
    y = data.y


if __name__ == "__main__":

    global data_laser, data_sick, th_target, x_target, y_target,x, y
    #初始化节点
    rospy.init_node("hc_avoid_obstacle")
    #监听激光数据和kinect数据
    rospy.Subscriber("/scan", LaserScan, avoid_obstacle)
    rospy.Subscriber("/KinectVision/direction", Direction, change_direction)
    thread = threading.Thread(target=read_data)
    thread1 = threading.Thread(target=read_kinect)
    thread.setDaemon(True)
    thread1.setDaemon(True)
    thread.start()
    thread1.start()
    ser = rospy.ServiceProxy("/hc_cmd_interface/vector_speed", VectorSpeed)
    time.sleep(1)
    base_r = 0.21 / cos(pi / 6)
    while True:
        base_r = 0.21 / cos(pi / 6)
        data_laser = data_sick
        x_target = x
        y_target = y
        th_target = asin(y_target/hypot(y_target, (x_target - 0.24)))
        print "x:%d" % x_target
        print "y:%d" % y_target
        print "角度:%d" %(-th_target / pi * 180)
        if len(data_laser) is 0:
            rospy.loginfo("等待sick数据")
            continue
        global th_find, n
        th_find = [5.0, -10.0, 20.0, -30.0, 45.0, -60.0]
        n = 0
        while x_target < 0:
            rospy.loginfo("丢失目标")
            print "th_find:%d" %th_find[n]
            print "trans:%d" %trans
            print (th_find[n] / 180 * pi / 2 / trans)
            ser(0, 0, th_find[n] / 180 * pi / 2 / trans)
            time.sleep(2)
            ser(0, 0, 0)
            time.sleep(5)
            n += 1
            if n >= 6:
                ser(0, 0, pi / 6 / 2 / trans)
                time.sleep(2)
                ser(800, 0, 0)
                time.sleep(3)
                n = 0
            x_target = x
            y_target = y
            th_target = asin(y_target/hypot(y_target, (x_target - 0.24)))
            print "x:%d" % x_target
            print "y:%d" % y_target
            print "角度:%d" % th_target
        #小于10度不转
        if abs(th_target / pi * 180) > 10:
            rospy.loginfo("转弯")
            print "角度:%d" %(-th_target / pi * 180)
            ser(0, 0, -th_target / 2 / trans)
            time.sleep(2)
            continue
        else:
            distance_target = hypot((x_target - 0.24), y_target)
            print "x:%d" % x_target
            print "y:%d" % y_target
            print "距离为%f" % distance_target
            if distance_target < 1.5:
                rospy.loginfo("太近了")
                ser(0, 0, 0)
                time.sleep(1)
                continue
            else:
                while manage_data(data_laser) is 0:
                    data_laser = data_sick

    rospy.spin()