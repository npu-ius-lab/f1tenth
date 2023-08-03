#!/usr/bin/env python
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
speed_param = rospy.get_param('~speed_param', 2) # 小车行驶速度
P_param = rospy.get_param('~P_param', 0.2) # 根据转角小车转弯减速的比例 

FILTER_VALUE = 3.0
P = 0.7
D = 0.3
THRESHOLD_obs = 0.3
MAX_DETECT_THRESHOLD =FILTER_VALUE
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
    
    Left_obs = []
    Left_obs_orig = []

    max_dis_num = []
    max_dis = 0 
    max_dir = 0
    max_dis_index = 0
    max_dir_index = 0
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
    #print(dis)
    #print(stop_num)
    if stop_num >12:
       stop_flag = stop_flag+1
    elif stop_flag != 6:
       stop_flag = 0
    #print(dis)
    dis_90 = dis
    lenth_dis = len(dis_90)

    for i in range(0,lenth_dis-2,1):
       if dis_90[i]-dis_90[i+1] > THRESHOLD_obs and len(Left_obs_orig)%2 ==0:
         Left_obs_orig.append(i+1)
       elif dis_90[i+1]-dis_90[i] > THRESHOLD_obs and len(Left_obs_orig)%2 ==1:
         Left_obs_orig.append(i)
    if len(Left_obs_orig)%2 == 1:
      Left_obs_orig.pop()


    if len(Left_obs_orig)>0 :
      for i in range(0,int(len(Left_obs_orig)/2),1):
        dis_obs_middle = dis_90[int((Left_obs_orig[2*i]+Left_obs_orig[2*i+1])/2)]
        dis_obs_sum = 0.0
        for j in range(Left_obs_orig[2*i],Left_obs_orig[2*i+1]):
          dis_obs_sum = dis_obs_sum + (dis_90[j]-dis_obs_middle) * (dis_90[j]-dis_obs_middle)
        if dis_obs_sum < 1:
          Left_obs.append(Left_obs_orig[2*i])
          Left_obs.append(Left_obs_orig[2*i+1])

    #print(Left_obs)
    if len(Left_obs)>0 :
      for i in range(0,int(len(Left_obs)/2),1):
        obs_middle =dis_90[int((Left_obs[2*i]+Left_obs[2*i+1])/2)]
        for j in range(int(max(Left_obs[2*i]-max((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))\
          ,int(min(Left_obs[2*i+1]+max((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1)),1):
            dis_90[j] = obs_middle


    for i in range(0,lenth_dis-2,1):
      if dis_90[i]<MAX_DETECT_THRESHOLD and dis_90[i+1] == MAX_DETECT_THRESHOLD and len(max_dis_num)%2==0:
        max_dis_num.append(i+1)
      if dis_90[i]==MAX_DETECT_THRESHOLD and dis_90[i+1] < MAX_DETECT_THRESHOLD and len(max_dis_num)%2==1:
        max_dis_num.append(i)
      if max_dis < dis_90[i] : 
         max_dis = dis_90[i]
         max_dis_index = i
    #print(max_dis_num)
    if len(max_dis_num)>1:
      max_dir_index = int((max_dis_num[0]+max_dis_num[1])/2)
      max_dir = max_dis_num[1]-max_dis_num[0]
      if len(max_dis_num)>2:
        for i in range(0,int(len(max_dis_num)/2),1):
         if max_dis_num[2*(i)+1]-max_dis_num[2*(i)] > max_dir:
           max_dir_index=int((max_dis_num[2*(i)+1]+max_dis_num[2*(i)])/2)
           max_dir = max_dis_num[2*(i)+1]-max_dis_num[2*(i)]
    if max_dir_index != 0:
        #print('22222')
        angle = -(max_dir_index-90)/450.0 * np.pi 
    else :
        #print('11111')
        angle = -(max_dis_index-90)/900.0 * np.pi

    
    steer_angle = P *( angle) +D * (angle-last_steer_angle)
    print(dis_90)
    #print(angle)
    #print('dir',max_dir_index)
    #print('dis',max_dis_index)

    # speed = 3.5
    speed = speed_param - P_param * abs(angle) - P_param * (angle-last_angle)*angle
    #print('speed=',speed)
    last_angle = angle
    last_steer_angle = steer_angle

    if stop_flag >=4:
       print('baohu')
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
