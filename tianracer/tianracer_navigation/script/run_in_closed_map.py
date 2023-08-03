#!/usr/bin/env python2.7
#coding=utf-8
# created by Wu GengQian

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

rospy.init_node("disparity_extender")
speed_param = rospy.get_param('~speed_param', 2.0) # 小车行驶速度
P_param = rospy.get_param('~P_param', 0.2) # 根据转角小车转弯减速的比例 

FILTER_VALUE = 10.0
P = 0.8
D = 0.4
global stop_flag 
global last_angle
global last_steer_angle
last_steer_angle = 0 
last_angle = 0
stop_flag = 0

# 获取激光雷达测量的距离
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

DISPARITY_DIF = 0.5 # 判断是否触碰障碍物的阈值
CAR_WIDTH = 0.3 # 小车宽度，从simulator的params.yaml获取

# 激光数据/scan的回调函数
def disparity_extender_callback(data):
    dis = []
    global stop_flag 
    global last_angle
    global last_steer_angle
    stop_num = 0
    #print(data.ranges[360:1080])
# 获取前向180°的测量距离
 
    for angle in range(0,180):
        if data.ranges[1080-abs(angle*4)]==data.range_min:
            dis.append(data.range_min)
        elif data.ranges[1080-abs(angle*4)]>FILTER_VALUE:
            dis.append(FILTER_VALUE)
        else:
            dis.append(data.ranges[1080-abs(angle*4)])
        if dis[angle] < 0.15 and angle > 80 and angle < 100:
            stop_num = stop_num+1
        if angle> 2 and dis[angle-1]==0.0:
           dis[angle-1] =(dis[angle-2])
    dis[179] = dis[178]
    if stop_num >=10:
       stop_flag = stop_flag+1
    elif stop_flag != 6:
       stop_flag = 0
    #print(dis)

 

    disparities = []
    for i in range(len(dis)):
        if i == len(dis) - 1:
            continue
        # 超过阈值则认为可能发生碰撞
        if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
            min_dis = min(dis[i], dis[i + 1]) # 计算可能碰撞最小距离
            if min_dis == 0:
                min_dis = min_dis + 0.1
            angle_range = math.ceil(math.degrees(math.atan(CAR_WIDTH / 2 / min_dis))) # 根据小车宽度计算不会碰撞的最小角度
            angle_range += 10 # 添加容差
            # 需要裁剪的激光数据范围
            side_range = range(int(i - angle_range + 1), i + 1) if dis[i + 1] == min_dis else range(i + 1, int(i + 1 + angle_range))
            disparities.append((min_dis, side_range))

    # 裁剪激光数据
    for min_dis, side_range in disparities:
        for i in side_range:
            if i >= 0 and i < len(dis):
                dis[i] = min(dis[i], min_dis)

    max_index = np.argmax(dis) # 得到最可行距离的角度
    max_dis = dis[max_index]
    angle_180 = (dis[10]-dis[169])/(dis[10]+dis[169])
    # 对角度进行限制，主要是避免激光数据抖动产生的影响
    #angle = max_index - 90 if abs(max_index - 90) > 15 else 0
    if max_index >150:
        max_index = 150
    elif max_index < 30:
        max_index = 30
    angle = max_index - 90
    angle = -angle * np.pi / 180 / 2 + angle_180 / 3
    angle = angle * abs(angle)
    steer_angle = P * angle +D * (angle-last_steer_angle)
    print(angle)
    # speed = 3.5
    speed = speed_param - P_param * abs(angle) - P_param * (angle-last_angle)*angle
    last_angle = angle
    last_steer_angle = steer_angle

    if stop_flag >=4:
       stop_flag = 6
       angle = 0
       speed = 0

      
    
    drive_msg = AckermannDrive()
    drive_msg.steering_angle=steer_angle
    drive_msg.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    scan_sub = rospy.Subscriber('/scan', LaserScan, disparity_extender_callback)
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
