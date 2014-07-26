#!/usr/bin/env python
#-*-encoding:utf-8-*-
#AUTHOR:jinzeshang
#NAME:hc_odom_control.py
#CREAT DATE:2014/7/25 22:13

import rospy
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *
from math import *
import threading

node_name = "hc_odom_control"	# 节点名字

# 上一次更新的码盘数据
pre_codedisk_position_of_M1 = 0
pre_codedisk_position_of_M2 = 0
pre_codedisk_position_of_M3 = 0
last_time = None
radius_of_wheel = 0.0625


# 机器人相关参数
#nam_speed = 0.0002333 #电机转一圈所走的距离
radius_of_base = 0.21 / sin( pi / 3)	# 底座半径
dis_per_unit = 2 * pi * radius_of_wheel / 4294967295


# 初始位置，该位置同时也决定了世界坐标系
x = 0.0
y = 0.0
th = 0.0
vx = 0.0
vy = 0.0
vth = 0.0

nam = pi / 3




# 由于码盘数据是循环值，两次码盘数据之差并不一定是真正的变化值
def get_diff(dis):
	if abs(dis)/100000 <= 40000 :
		if dis < 0:
			dis = dis + 4294967295
		else:
			dis = dis - 4294967295
	else:
		dis=0
	return dis

# 获取码盘数据，计算和更新历程数据
def handle_codedisk_data(codedisk_data):
#rospy.loginfo("test")
	global inverse_matrix, last_time
	global x, y, th, vx, vy, vth
	global pre_codedisk_position_of_M1, pre_codedisk_position_of_M2, pre_codedisk_position_of_M3

# 获取数据
	codedisk_position_of_M1 = codedisk_data.data1
	codedisk_position_of_M2 = codedisk_data.data2
	codedisk_position_of_M3 = codedisk_data.data3
	current_time = rospy.get_time()
#print last_time
#print current_time

# 并非是第一次获取码盘值
	if last_time != None:
# 获取码盘变化值
		dis_value1 = get_diff(codedisk_position_of_M1 - pre_codedisk_position_of_M1)
		dis_value2 = get_diff(codedisk_position_of_M2 - pre_codedisk_position_of_M2)
		dis_value3 = get_diff(codedisk_position_of_M3 - pre_codedisk_position_of_M3)
#		print "wtf"


# 起码有一个码盘发生数据变化时候才计算里程
		if abs(dis_value1) < 3:
			dis_value1 = 0
		if abs(dis_value2) < 3:
			dis_value2 = 0
		if abs(dis_value3) < 3:
			dis_value3 = 0
		if abs(dis_value1) > 2 or abs(dis_value2) > 2 or abs(dis_value3) > 2 :
			print "码盘一：", dis_value1
			print "码盘二：", dis_value2
			print "码盘三：", dis_value3
	
# 根据轮子的半径和周长，计算运动的距离
			dis1 = dis_per_unit * dis_value1
			dis2 = dis_per_unit * dis_value2
			dis3 = dis_per_unit * dis_value3

# 根据轮子的运动距离，计算 v1, v2 和 v3
			time_change = (current_time - last_time)
			v1 = dis1 / time_change
			v2 = dis2 / time_change
			v3 = dis3 / time_change
			print "v1 = ", v1, "v2 = ", v2, "v3 = ", v3

# 重新计算 vx, vy 和 vz，后面参数叫矫正值
			vx = (v1-v2)/(2*sin(nam))
			vy = (v1+v2-2*v3)/(2+2*cos(nam))
			omega = (v1+v2+2*v3*cos(nam))/(2*radius_of_base+2*radius_of_base*cos(nam))
			print "机器人坐标系速度：", vx, vy, omega
			(dx, dy, dth) = cal_odom(vx, vy, omega, th, time_change)
			print "里程变化量：", dx, dy, dth * 180 / pi

# 计算世界坐标系中的vx 和 vy
			vx_in_w = vx * cos(-th) - vy * sin(-th)
			vy_in_w = vx * sin(-th) + vy * cos(-th)
			print "世界坐标系中的速度：", vx_in_w, vy_in_w

# 当前位置（里程）
			x = x + dx
			y = y + dy
			th = th + dth * 1.07
			print "当前位置：", x, y, th * 180 / pi
			print ""

# 存储
	pre_codedisk_position_of_M1 = codedisk_position_of_M1
	pre_codedisk_position_of_M2 = codedisk_position_of_M2
	pre_codedisk_position_of_M3 = codedisk_position_of_M3
	last_time = current_time
	#print current_time

# 里程数据计算
def cal_odom(vx, vy, omega, th, dt):
	vx_in_w = vx * cos(th) - vy * sin(th)
	vy_in_w = vx * sin(th) + vy * cos(th)
#dx = vx_in_w * dt
#dy = vy_in_w * dt
#dth = omega * dt
#return (dx, dy, dth)

# 无自旋的情况

	if abs(omega) <= 0.1:
		rospy.logerr("直线运动", omega / pi * 180)
# 在世界坐标系中的速度
#vx_in_w = vx * cos(-th) - vy * sin(-th)
#vy_in_w = vx * sin(-th) + vy * cos(-th)
# 在世界坐标系中的位移
		dx = vx_in_w * dt
		dy = vy_in_w * dt
		dth = 0
		print dx 
		print dy 
		print dth
		return (dx, dy, dth)
# 有自转情况下
	else:
		rospy.logerr("曲线运动", omega / pi * 180)
		dth = omega * dt
		dy = vy_in_w * dt
		dx = vx_in_w * dt
		print dx 
		print dy 
		print dth
		return (dx, dy, dth)
	

if __name__ == "__main__":
# 初始化节点
	rospy.init_node(node_name)
	print "准备监听"
# 监听码盘数据，计算当前位置
	rospy.Subscriber("/hc_cmd_interface/code_disk", CodeDisk, handle_codedisk_data)
# rospy.Subscriber("/scan_2", LaserScan, handle_scan_data)


# 等待
	rospy.spin()
