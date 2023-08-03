#! /usr/bin/env python3c
#coding=utf-8
# Created by Yao Dexin
# Modified by HuangHao
from dis import dis
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

global last_angle
last_angle = 0
MAX_DETECT_THRESHOLD = 3.5
THRESHOLD_obs = 0.15
START_ANGLE = -60
END_ANGLE = 60
P = 0.80
D = 0.25
# def get_dis(data, angle, deg=True):
#     if deg:
#         angle = np.deg2rad(angle)
#     #print("angle min is ",data.angle_min)
#     print("angle incre is ",data.angle_increment)
#     dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
#     return dis

def get_dis(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    #print("angle min is ",data.angle_min)
    #print("angle incre is ",data.angle_increment)
    dis = 0
    temp = int((angle - data.angle_min) / data.angle_increment)
    for i in range (temp, temp + 4, 1):
      dis =  dis + data.ranges[i]
    dis = dis / 4.0
    #dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    return dis

def get_range(data, start_angle, end_engle):
    all_dis = []
    for angle in range(start_angle,end_engle):
      all_dis.append(get_dis(data, angle))
    return all_dis


def middle_line_callback(data):
    global last_angle
    print(len(data.ranges))
    dis = get_range(data, -90,90)# 前方180度雷达数据
    dis_90 = dis[::-1]
    #print(dis_90)
    lenth_dis = len(dis_90)
    dis_90_copy = tuple(dis_90)
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
    print("before inflation is",Left_obs)
    Left_obs_form = tuple(Left_obs)
    #以下代码为 障碍膨胀 将原来的障碍区间进行膨胀再找最大可行方向 留一定的余量
    if len(Left_obs)>0 :#有障碍才进入
      for i in range(0,int(len(Left_obs)/2),1):
        obs_middle =dis_90[int((Left_obs[2*i]+Left_obs[2*i+1])/2)]
        for j in range(int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))\
          ,int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1)),1):
            dis_90[j] = obs_middle
        Left_obs[2*i]=int(max(Left_obs[2*i]-min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),0))
        Left_obs[2*i+1]=int(min(Left_obs[2*i+1]+min((Left_obs[2*i+1]-Left_obs[2*i])/2*(4-obs_middle),10),lenth_dis-1))
    print("after inflation  is",Left_obs)
    #以下代码为找有障碍
    if len(Left_obs)>0 :
      for i in range(0,int(len(Left_obs)/2)+1,1):
        if i == 0:
          for j in range(Left_obs[0]-1,0,-1):
            if dis_90[j]<=dis_90[Left_obs[0]+1]:
              #print('sssss')
              max_dis_num.append(j)
              max_dis_num.append(Left_obs[0]+1)
              break
          if len(max_dis_num)==0:
            #print('sssssacawqdqws')
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
      max_dis_index_temp = max_dis_index
      #以下代码基于上面代码块计算的障碍间隙数组进行处理  计算目标方向
      for i in range(0,int(len(max_dis_num)/2),1):
        if max_dis_val < max_dis_num[2*i+1] - max_dis_num[2*i] :
          max_dis_val = max_dis_num[2*i+1] - max_dis_num[2*i]
          max_dis_index = (max_dis_num[2*i+1] + max_dis_num[2*i])/2
        if left == 1 and max_dis_index < 90 and dis_90[0]<dis_90[lenth_dis-1]-1:
          max_dis_index = max_dis_index+5*abs(dis_90[lenth_dis-1]-dis_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
        elif right == 1 and max_dis_index > 90 and dis_90[0]-1>dis_90[lenth_dis-1]:
          max_dis_index = max_dis_index-5*abs(dis_90[lenth_dis-1]-dis_90[0])#作补偿 因为目标方向和最大距离方向不一致  可以进行限幅
      #print("最远方向",max_dis_index_2)
      #print("有障碍 max dis index",max_dis_index)
      '''mid = list()
      res = 180
      max_dis_index_2 = 89
      for i in range(int(len(max_dis_num)/2)):
        if max_dis_num[2*i+1] - max_dis_num[2*i] > 10:#大于10度认为可以通过
          #mid.append(abs(int((max_dis_num[2*i+1] + max_dis_num[2*i])/2)-max_dis_index_2))
          mid = abs(int((max_dis_num[2*i+1] + max_dis_num[2*i])/2)-max_dis_index_2)
          if res > mid:
            res = mid
            res_index = i
      max_dis_index = (max_dis_num[2*res_index+1] + max_dis_num[2*res_index])/2
      print("after my compute",max_dis_index)'''
      #比中间
      print("max_dis_index_temp",max_dis_index_temp)
      if len(Left_obs_form) ==2 :#一个障碍:
        if max_dis_index_temp >=89:#右转
          print("turn right")
          if Left_obs_form[0] >=89:
            max_dis_index = int((89+Left_obs[0])/2)
          elif Left_obs_form[1] <89:
            max_dis_index = max_dis_index_temp
          else:#正前方情况，不太合理
            max_dis_index = max_dis_index_temp
        else:#左转
          print("turn left")
          if Left_obs_form[0] >=89:
            print("1")
            max_dis_index = max_dis_index_temp #######
          elif Left_obs_form[1] <=89:
            print("2")
            max_dis_index = int((89+Left_obs[1])/2)
          else:
            print("3")
            max_dis_index = max_dis_index_temp
      if len(Left_obs_form) ==4: #两个障碍
        middle_temp = int((Left_obs[1]+Left_obs[2])/2)
        if max_dis_index_temp >=89:#右转
          if Left_obs_form[0] >= 89:
            max_dis_index = int((89+Left_obs[0])/2)
          elif Left_obs_form[1] <= 89 and middle_temp >89:
            #max_dis_index = int((Left_obs[1]+Left_obs[2])/2)
            max_dis_index = int((Left_obs[1]+max_dis_index_temp)/2)
          elif middle_temp <= 89 and Left_obs[2] >89:
            max_dis_index = middle_temp
          else:
            max_dis_index = max_dis_index_temp
        else:
          print("turn left")
          if Left_obs_form[0] > 89:
            max_dis_index = max_dis_index_temp
          elif Left_obs_form[0] <= 89 and middle_temp >89:
            max_dis_index = middle_temp
          elif middle_temp <= 89 and Left_obs_form[3] >89:
            max_dis_index = middle_temp#int((Left_obs[2]+max_dis_index_temp)/2)
          else:
            max_dis_index = int((Left_obs[3] + 89)/2)

      
      #print("after my compute",max_dis_index)
      print("after compute max_dis_index is",max_dis_index)
    #以下代码寻找最大距离（阈值距离）区域
    for i in range(0,lenth_dis-2,1):
      if dis_90[i]<MAX_DETECT_THRESHOLD and dis_90[i+1] == MAX_DETECT_THRESHOLD and len(max_dir_num)%2==0:
        max_dir_num.append(i+1)
      elif dis_90[i]==MAX_DETECT_THRESHOLD and dis_90[i+1] < MAX_DETECT_THRESHOLD and len(max_dir_num)%2==1:
        max_dir_num.append(i)
    #print("最大距离数组max dir num is",max_dir_num)
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
    #print("最大距离方向 max dir index",max_dir_index)
    #print("dis_90[0] is",dis_90[0])    
    #print("dis_90[lenth_dis-1] is",dis_90[lenth_dis-1])
    dis_90[0] = dis_90[0] +0.00001
    print("max dis is",max_dis)
    print("max dis index used for compute is",max_dis_index)
    print("dis_90[0] and dis_90[len-1] is",[dis_90[0],dis_90[lenth_dis-1]])
    dis_90[lenth_dis-1] = dis_90[lenth_dis-1] +0.00001
    if max_dir_index != 0 and len(Left_obs) == 0:
       if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3:
         print("333")
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi + 0.1*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
       else:
         print("444")
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dir_index-90)/360 *math.pi + 0.05*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
        #  print(max_dir_index,Left_obs,max_dir_num,max_dis_num,max_dis_index)
    else :
       if dis_90[0]/dis_90[lenth_dis-1]>3 or dis_90[lenth_dis-1]/dis_90[0]>3:
         print("111")
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi + 0.1*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1])
       else:
         print("222")
         angle = -max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi + 0.05*(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1]) 
        #  print(Left_obs,max_dis_num,max_dis_index,left)
    
    # print(dis)
    print("angle1 is ",angle*180/3.14)
    print("last_angle is ",last_angle*180/3.14)
    print("-max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi is",-max(math.exp(-max_dis/MAX_DETECT_THRESHOLD),0.7)*(max_dis_index-90)/360 *math.pi)
    print("(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1]) ",(dis_90[0]-dis_90[lenth_dis-1])/(dis_90[0]+dis_90[lenth_dis-1]) )
    steering_angle = P* angle +D *(angle-last_angle)
    last_angle = angle
    #print("angle is",angle)
    if angle > 0:
      print("steering left %f"%(abs(steering_angle*180/math.pi)))
    else:
      print("steering right %f"%(abs(steering_angle*180/math.pi)))
    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    #speed = 2.8*(0.3*math.exp(-np.clip(abs(angle),0,0.5))+0.7)
    absangle = abs(angle)
    speed = -333.3333*pow(absangle,5)+353.7296*pow(absangle,4)-90.6469*pow(absangle,3)-8.9962*pow(absangle,2)+0.3738*absangle+3.1912
    speed = 1.05*speed
    angle_filter = steering_angle
    #print("speed is ",speed)
    max_dis = []
    mean_max_dis=0
    #print("max dis index",max_dir_index)
    if max_dir_index!=0 and len(Left_obs) ==0:
      for i in range(90-2,90+4,1):
        max_dis.append(dis_90_copy[i])
      #print("angle is",angle)
      #print("最远距离",max_dis)
      mean_max_dis = np.mean(max_dis)
      if mean_max_dis > 5.5:#前面8米没东西
        speed = speed*1.2
    print(speed)
    drive_msg = AckermannDrive()
    drive_msg.steering_angle=steering_angle
    drive_msg.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, middle_line_callback,queue_size=1)
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
