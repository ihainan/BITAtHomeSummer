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
from bitathome_hardware_control.srv import *

data_sick = []
data_laser = []
threshold = [3, 2, 1]  # 不同的阈值
base_r = 0.21 / cos(pi / 6)  # 底座半径
trans = 0.00023  # 轮子转速为１的时候实际运动速度为0.00023


#求可行方向
def find_dir(dis, r):
    sectors = []  # 存放可用的扇区
    sec = []  #备份扇区
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
        th = ((sectors[i][0] + sectors[i][1] + 1) * 3 - 180) / 2 * pi / 180
        sectors[i].append(th)
    #验证是否满足条件
    k = 0
    for i in range(0, len(sec)):
        #弦长条件
        if base_r * 1.2 > r * sin((sec[i][1] - sec[i][0] + 1) * 3 * pi / 180 / 2):
            del sectors[i - k]
            k += 1
            continue
        #矩形条件
        d = r * cos((sec[i][1] - sec[i][0] + 1) * 3 * pi / 180 / 2)  #矩形的长
        #角度范围
        th1 = sec[i][0] * 3 * pi / 180 - pi / 2
        th2 = (sec[i][1] + 1) * 3 * pi / 180 - pi / 2
        if sec[i][2] <= 0:
            for j in range(0, len(dis)):
                th = (j * 3 - 90) * pi / 180  # 第ｊ个点的角度
                if th > sec[i][2] + pi / 2:
                    break
                elif th < 0 and (th < th1 or th > th2):
                    if dis[j] < base_r * 1.2 / sin(sec[i][2] - th):  # 有点落在矩形内
                        del sectors[i - k]
                        k += 1
                        break
                elif th > 0 and (th < th1 or th > th2):
                    if dis[j] < base_r * 1.2 / cos(sec[i][2] + pi / 2 - th):
                        del sectors[i - k]
                        k += 1
                        break
        else:
            for j in range(0, len(dis)):
                th = (j * 3 - 90) * pi / 180  # 第ｊ个点的角度
                if th < sec[i][2] - pi / 2:
                    continue
                elif th < 0 and (th < th1 or th > th2):
                    if dis[j] < base_r * 1.2 / sin(sec[i][2] - th):  # 有点落在矩形内
                        del sectors[i - k]
                        k += 1
                        break
                elif th > 0 and (th < th1 or th > th2):
                    if dis[j] < base_r * 1.2 / cos(sec[i][2] + pi / 2 - th):
                        del sectors[i - k]
                        k += 1
                        break
    return sectors


#处理数据
def manage_data(data_laser):
    distance = []
    #取-90~90度，将它分为60组，每组六个数据
    for i in range(91, 451, 6):
        distance.append(data_laser[i: i+6])
    #取每组中的最小值作为这组的标记值并存放到dis中
    dis = []
    tmp = 0
    for i in distance:
        for j in i:
            tmp = j
            if j < tmp and j > 0.009:
                tmp = j
        if tmp < 0.009:
            tmp = 3
        dis.append(tmp)
    sec = []  # 用于存放最大的阈值得到的可行方向
    R = 0
    for r in threshold:
        sec = find_dir(dis, r)
        if len(sec) is not 0:
            R = r
            break
    if len(sec) is 0:
        rospy.loginfo("没找着可行方向")
        ser(0, 0, 400)
        time.sleep(2)
    else:
        th_best = sec[0][2]
        for i in sec:
            print sec
            if abs(i[2]) < abs(th_best):
                th_best = i[2]
        th_best = -th_best
        print th_best

        print (th_best / 2.5 / trans)
        if th_best / 2.5 / trans > 2000:
            ser(0, 0, 2000)
            time.sleep(2)
            ser(R * 300, 0, 0)
        else:
            if abs(th_best) < 10 * pi / 180:  #低于10度不转
                ser(R * 300, 0, 0)
                time.sleep(2)
            else:
                ser(0, 0, th_best / 2.5 / trans)
                time.sleep(2)
                ser(R * 300, 0, 0)


#读取数据
def read_data():
    rospy.Subscriber("/scan", LaserScan, avoid_obstacle)
    rospy.spin()


#将数据存到data_laser
def avoid_obstacle(data):
    global data_sick
    data_sick = data.ranges


if __name__ == "__main__":
    global data_laser, data_sick
    #初始化节点
    rospy.init_node("hc_avoid_obstacle")
    #监听激光数据
    rospy.Subscriber("/scan", LaserScan, avoid_obstacle)
    thread = threading.Thread(target=read_data)
    thread.setDaemon(True)
    thread.start()
    ser = rospy.ServiceProxy("/hc_cmd_interface/vector_speed", VectorSpeed)
    time.sleep(1)
    while True:
        data_laser = data_sick
        if len(data_laser) is 0:
            rospy.loginfo("等待sick数据")
            continue
        manage_data(data_laser)
    rospy.spin()