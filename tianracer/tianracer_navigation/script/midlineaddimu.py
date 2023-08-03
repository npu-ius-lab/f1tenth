#! /usr/bin/env python3
#coding=utf-8
# Created by Yao Dexin
# Modified by HuangHao

from dis import dis
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan,Imu
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
import message_filters
#from tf.transformations import euler_from_quaternion

global last_angle
last_angle = 0
MAX_DETECT_THRESHOLD = 3.5
THRESHOLD_obs = 0.15
START_ANGLE = -60
END_ANGLE = 60
P = 1.0
D = 0.3 
def qtoeu(x,y,z,w):
  r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
  r = r / math.pi * 180
  p = math.asin(2 * (w * y - z * x))
  p = p / math.pi * 180
  y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  y = y / math.pi * 180
  return [r,p,y]
def get_dis(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    #print("angle min is ",data.angle_min)
    #print("angle incre is ",data.angle_increment)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    return dis

def get_range(data, start_angle, end_engle):
    all_dis = []
    for angle in range(start_angle,end_engle):
      all_dis.append(get_dis(data, angle))
    return all_dis


def middle_line_callback(data1,data2):
    global last_angle
    dis = get_range(data1, -90,90)# 前方180度雷达数据
    dis_90 = dis[::-1]
    #print(dis_90)
    lenth_dis = len(dis_90)

    left = 0
    right = 0
    Left_obs_orig = []
    Left_obs = []
    max_dis_num = []
    max_dir_num = []
    max_dis = 0 
    max_dir = 0
    max_dis_index = 0
    max_dir_index = 0
    #print(dis_90)
    #以下代码为原始数据处理，作用找最大距离方向 以及数据限幅
    for i in range(0,lenth_dis-2,1):
        if dis_90[i]>max_dis and i>20 and i<160:
           max_dis = dis_90[i]
           max_dis_index = i 
        if  dis_90[i] > MAX_DETECT_THRESHOLD:
          dis_90[i] = MAX_DETECT_THRESHOLD
    #以下代码为了判断前方方向（根据最远距离的角度下标进行判断
    max_dis_index_2 = max_dis_index
    if max_dis_index <89:
      left = 1
    else:
      right =1
    
    ############################、
    #障碍物数据获取+滤波
    #以下代码为找原始障碍区间
    for i in range(len(dis_90)):
      if dis_90[i]==0:
        dis_90[i] = dis_90[i-1]
    for i in range(len(dis_90)):
      if dis_90[i]==0:
        print("zero is ???????",i)
     
    for i in range(0,lenth_dis-2,1):
       if dis_90[i]-dis_90[i+1] > THRESHOLD_obs and len(Left_obs_orig)%2 ==0:
         #print("进点是",i)
         Left_obs_orig.append(i+1)
       elif dis_90[i+1]-dis_90[i] > THRESHOLD_obs and len(Left_obs_orig)%2 ==1:
         #print("出点是",i)
         Left_obs_orig.append(i)
    #障碍范围总是成对出现进行一次滤波
    if len(Left_obs_orig)%2 == 1:
      Left_obs_orig.pop()
    #障碍范围总是成对出现进行二次滤波 计算所有障碍区间的方（区间所有数据与区间最中间值方差）  小于1才看作障碍
    #还可以加入判断区间大小限制  区间太小视为误判  删除
    # print(max_dis_index)
    if len(Left_obs_orig)>0 :
      for i in range(0,int(len(Left_obs_orig)/2),1):
        dis_obs_middle = dis_90[int((Left_obs_orig[2*i]+Left_obs_orig[2*i+1])/2)]
        dis_obs_sum = 0.0
        for j in range(Left_obs_orig[2*i],Left_obs_orig[2*i+1],1):
          dis_obs_sum = dis_obs_sum+(dis_90[j]-dis_obs_middle)*(dis_90[j]-dis_obs_middle)
        # print(dis_obs_sum)
        if dis_obs_sum < 1 :
          Left_obs.append(Left_obs_orig[2*i])
          Left_obs.append(Left_obs_orig[2*i+1])
    # print(dis)  
    #滤掉相同的点
    
    if len(Left_obs) >0:
      for i in range(int(len(Left_obs)/2)):
        if abs(Left_obs[2*i] - Left_obs[2*i+1])<2:
          Left_obs[2*i] = -1
          Left_obs[2*i+1] = -1
    Left_obs_temp = Left_obs
    Left_obs = list()
    for i in range(len(Left_obs_temp)):
      if Left_obs_temp[i] != -1:
        Left_obs.append(Left_obs_temp[i])
    
    #以下代码为 障碍膨胀 将原来的障碍区间进行膨胀再找最大可行方向 留一定的余量
    if len(Left_obs)>0 :#有障碍才进入
      for i in range(0,int(len(Left_obs)/2),1):
        obs_middle =dis_90[int((Left_obs[2*i]+Left_obs[2*i+1])/2)]
        for j in range(int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))\
          ,int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1)),1):
            dis_90[j] = obs_middle
        Left_obs[2*i]=int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))
        Left_obs[2*i+1]=int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1))
    print("膨胀之后 is",Left_obs)
    #以下代码为找有障碍
    if len(Left_obs)>0 :
      for i in range(0,int(len(Left_obs)/2)+1,1):
        if i == 0:
          for j in range(Left_obs[0]-1,0,-1):
            if dis_90[j]<=dis_90[Left_obs[0]+1]:
              print('sssss')
              max_dis_num.append(j)
              max_dis_num.append(Left_obs[0]+1)
              break
          if len(max_dis_num)==0:
            print('sssssacawqdqws')
            for j in range(0,Left_obs[0]-1,1):
              if dis_90[j]>=dis_90[Left_obs[0]+1]:
                max_dis_num.append(j)
                max_dis_num.append(Left_obs[0]+1)
                break
        elif i < int(len(Left_obs)/2):
          max_dis_num.append(Left_obs[2*i-1])
          max_dis_num.append(Left_obs[2*i])
        elif i == int(len(Left_obs)/2):
          for j in range(Left_obs[2*i-1]+1,lenth_dis-1,1):
            if dis_90[j]<=dis_90[Left_obs[2*i-1]-1]:
              max_dis_num.append(Left_obs[2*i-1]-1)
              max_dis_num.append(j)
              break
          if len(max_dis_num) != 2*i+1:
            for j in range(lenth_dis-1,Left_obs[2*i-1]+1,-1):
              if dis_90[j]>=dis_90[Left_obs[2*i-1]-1]:
                max_dis_num.append(Left_obs[2*i-1]-1)
                max_dis_num.append(j)
                break
      print("max_dis_num is",max_dis_num)
      max_dis_val = 0
      #以下代码基于上面代码块计算的障碍间隙数组进行处理  计算目标方向
      for i in range(0,int(len(max_dis_num)/2),1):
        if max_dis_val < max_dis_num[2*i+1] - max_dis_num[2*i] :
          max_dis_val = max_dis_num[2*i+1] - max_dis_num[2*i]
          max_dis_index = (max_dis_num[2*i+1] + max_dis_num[2*i])/2
        if left == 1 and max_dis_index < 90 and dis_90[0]<dis_90[lenth_dis-1]-1:
          max_dis_index = max_dis_index+5*abs(dis_90[lenth_dis-1]-dis_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
        elif right == 1 and max_dis_index > 90 and dis_90[0]-1>dis_90[lenth_dis-1]:
          max_dis_index = max_dis_index-5*abs(dis_90[lenth_dis-1]-dis_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
      print("最远方向",max_dis_index_2)
      print("有障碍 max dis index",max_dis_index)
      '''mid = list()
      res = 180
      for i in range(int(len(max_dis_num)/2)):
        if max_dis_num[2*i+1] - max_dis_num[2*i] > 10:#大于10度认为可以通过
          #mid.append(abs(int((max_dis_num[2*i+1] + max_dis_num[2*i])/2)-max_dis_index_2))
          mid = abs(int((max_dis_num[2*i+1] + max_dis_num[2*i])/2)-max_dis_index_2)
          if res > mid:
            res = mid
            res_index = i
      max_dis_index = (max_dis_num[2*res_index+1] + max_dis_num[2*res_index])/2
      print("after my compute",max_dis_index)'''

    #以下代码寻找最大距离（阈值距离）区域
    for i in range(0,lenth_dis-2,1):
      if dis_90[i]<MAX_DETECT_THRESHOLD and dis_90[i+1] == MAX_DETECT_THRESHOLD and len(max_dir_num)%2==0:
        max_dir_num.append(i+1)
      elif dis_90[i]==MAX_DETECT_THRESHOLD and dis_90[i+1] < MAX_DETECT_THRESHOLD and len(max_dir_num)%2==1:
        max_dir_num.append(i)
    print("最大距离数组max dir num is",max_dir_num)
    #如果找到最大距离（阈值距离）区域   取最大距离（阈值距离）区域中间的下标作为前进方向  
    if len(max_dir_num)>1:
      if abs(max_dir_num[0]-max_dir_num[1])>4:
        max_dir_index = int((max_dir_num[0]+max_dir_num[1])/2)
        max_dir = max_dir_num[1]-max_dir_num[0]
      if len(max_dir_num)>2:
        for i in range(0,int(len(max_dir_num)/2),1):
         if max_dir_num[2*(i)+1]-max_dir_num[2*(i)] > max_dir:
           max_dir_index=int((max_dir_num[2*(i)+1]+max_dir_num[2*(i)])/2)
           max_dir = max_dir_num[2*(i)+1]-max_dir_num[2*(i)]
    for i in range(0,len(Left_obs)-1,1):
      if abs(max_dir_index-Left_obs[i])<20*min(1/(dis_90[Left_obs[i]]+0.001),2): #######有问题Left_obs[i]->dis_90[Left_obs[i]]
        max_dir_index=0
        break
    print("最大距离方向max dir index",max_dir_index)
    #print("dis_90[0] is",dis_90[0])    
    #print("dis_90[lenth_dis-1] is",dis_90[lenth_dis-1])
    dis_90[0] = dis_90[0] +0.00001
    print("max dis is",max_dis)
    print("dis_90[0] and dis_90[len-1] is",[dis_90[0],dis_90[lenth_dis-1]])
    dis_90[lenth_dis-1] = dis_90[lenth_dis-1] +0.00001
    if max_dir_index != 0:
       if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3:
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi \
           + 0.1*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
       else:
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi \
           + 0.05*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
        #  print(max_dir_index,Left_obs,max_dir_num,max_dis_num,max_dis_index)
    else :
       if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3:
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi \
            + 0.1*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
       else:
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi \
           + 0.05*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
        #  print(Left_obs,max_dis_num,max_dis_index,left)
    
    # print(dis)
       
    print("angle1 is ",angle)
    steering_angle = P* angle +D *(angle-last_angle)
    last_angle = angle
    print("angle is",angle)
    if angle > 0:
      print("向左转%f度"%(abs(angle*180/math.pi)))
    else:
      print("向右转%f度"%(abs(angle*180/math.pi)))
    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    speed = 2*(0.3*math.exp(-np.clip(abs(angle),0,0.5))+0.7)
    angle_filter = steering_angle
    #print("speed is ",speed)
    theta = qtoeu(data2.orientation.x,data2.orientation.y,data2.orientation.z,data2.orientation.w)
    print("jiaodu shi is",theta[2])
    drive_msg = AckermannDrive()
    print(steering_angle)
    drive_msg.steering_angle=0
    drive_msg.speed=0
    
    print(speed)
    drive_pub.publish(drive_msg)

def sub_two_topic():
    t1= message_filters.Subscriber("/scan", LaserScan)
    t2 =message_filters.Subscriber("/tianracer/imu",Imu)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 1, 1, allow_headerless=True)
    ts.registerCallback(middle_line_callback)
    rospy.spin()


if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
    sub_two_topic()

    
  except rospy.ROSInterruptException:
    pass
