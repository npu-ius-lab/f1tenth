#! /usr/bin/env python
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

class DisparityExtenderDriving(object):

    #constructor for our DisparityExtenderDrivng Object
    #stores configuration parameters neccessary for successful execution of our algorithm
    def __init__(self,decoupled=False):

        self.MAX_DETECT_THRESHOLD = 4.0
        self.THRESHOLD_obs = 0.35
        self.START_ANGLE = -60
        self.END_ANGLE = 60
        self.P = 0.80
        self.D = 0.25

        self.left = 0
        self.right = 0

        self.max_dis = 0 
        self.max_dir = 0

        self.max_dis_index = 0
        self.max_dir_index = 0
        
        self.Left_obs_orig = []
        self.Left_obs = []
        self.max_dis_num = []
        self.max_dir_num = []
        
        self.dis_obs_middle = 0
        
        self.drive_pub = rospy.Publisher( '/drive', AckermannDriveStamped, queue_size=1) 

    #计算距离
    def get_dis(self,data, angle, deg=True):
      #角度转换为弧度
        if deg:
            angle = np.deg2rad(angle)
        dis = 0
        #计算当前激光的序列, data.angle_min是激光扫描的最小角度, data.angle_increment 是激光扫描角度的增量
        temp = int((angle - data.angle_min) / data.angle_increment)
        # 将激光的数据按序列加进去并取平均
        for i in range (temp, temp + 4, 1):
          dis =  dis + data.ranges[i]
        dis = dis / 4.0
        return dis
              
    #限制角度
    def get_range(self,data, start_angle, end_engle):
        all_dis = []
        for angle in range(start_angle,end_engle):
          all_dis.append(self.get_dis(data, angle))
        return all_dis

    def find_max_dis(self,lenth_dis,dis_90):
        #以下代码为原始数据处理，作用找最大距离和所在的序号，并对最远距离进行限幅
        for i in range(0,lenth_dis-2,1):
            if dis_90[i]>self.max_dis and i>20 and i<160:
              self.max_dis = dis_90[i]
              self.max_dis_index = i 
            if  dis_90[i] > self.MAX_DETECT_THRESHOLD:
              dis_90[i] = self.MAX_DETECT_THRESHOLD

        return dis_90

    #将列表中的零值替换为前一个非零元素的值
    def exchange(self,dis_90):
        for i in range(len(dis_90)):
          if dis_90[i]==0:
            dis_90[i] = dis_90[i-1]
            
        return dis_90
    
    #从 dis_90 数据中找到连续的障碍物区间并滤波
    def obstacle_intervals(self,lenth_dis,dis_90):
        for i in range(0,lenth_dis-2,1):
          #落差大于阈值且列表 self.Left_obs_orig 中的元素个数为偶数
          if dis_90[i]-dis_90[i+1] > self.THRESHOLD_obs and len(self.Left_obs_orig)%2 ==0:
            #添加新的障碍物区间，进点
            self.Left_obs_orig.append(i+1)
          #落差大于阈值且列表 self.Left_obs_orig 中的元素个数为奇数
          elif dis_90[i+1]-dis_90[i] > self.THRESHOLD_obs and len(self.Left_obs_orig)%2 ==1:
            #障碍物区间闭合，出点
            self.Left_obs_orig.append(i)
        #障碍范围总是成对出现，进行一次滤波
        if len(self.Left_obs_orig)%2 == 1:
          self.Left_obs_orig.pop()
 
 
    # 计算所有障碍区间的方差，小于1才看作障碍         
    def obstacle_deviation(self,dis_90):
        if len(self.Left_obs_orig) > 0:
            processed_obs = []
            
            # 以步长为2来进行遍历记录的障碍物区间
            for i in range(0, len(self.Left_obs_orig), 2):
                # 障碍物中间值
                self.dis_obs_middle = dis_90[int((self.Left_obs_orig[i] + self.Left_obs_orig[i + 1]) / 2)]
                # 计算障碍物范围内每个距离值与中间点距离的平方，并将它们求和
                dis_obs_sum = sum([(dis_90[j] - self.dis_obs_middle) ** 2 for j in range(self.Left_obs_orig[i], self.Left_obs_orig[i + 1])])
                
                # 检查dis_obs_sum是否小于1，是的话障碍物范围内的距离值变化不大，被认为是一个障碍物
                if dis_obs_sum < 1:
                    processed_obs.extend([self.Left_obs_orig[i], self.Left_obs_orig[i + 1]])

            # 保留障碍物范围差距大于等于2的元素。
            filtered_obs = [processed_obs[i] for i in range(0, len(processed_obs), 2)
                            if abs(processed_obs[i] - processed_obs[i + 1]) >= 2]

            self.Left_obs = [obs for obs in filtered_obs if obs != -1]
            print("processed obstacles:", self.Left_obs)
    
     #障碍物膨胀       
    def obstacle_inflate(self,dis_90,lenth_dis):
        if len(self.Left_obs) >= 2 and len(self.Left_obs)%2 == 0:
            inflated_obs = []
            
            for i in range(0, len(self.Left_obs), 2):

                # 计算膨胀系数并得到左右边界
                inflate_dist = min((self.Left_obs[i + 1] - self.Left_obs[i]) / 2 * (4 - self.dis_obs_middle), 10)
                left_bound = max(self.Left_obs[i] - inflate_dist, 0)
                right_bound = min(self.Left_obs[i + 1] + inflate_dist, lenth_dis - 1)
                
                # 在膨胀边界内的障碍物距离统一设置为障碍物中间点的距离
                for j in range(int(left_bound), int(right_bound) + 1):
                    dis_90[j] = self.dis_obs_middle
                
                inflated_obs.extend([int(left_bound), int(right_bound)])
            
            # 膨胀后的数据进行替换
            self.Left_obs = inflated_obs

            print("after inflation is", self.Left_obs)

    def direction_between_obstacles (self,dis_90,lenth_dis):
        # 存在一个或多个障碍
        # 遍历self.Left_obs列表中的障碍物区间，找到具有障碍物的位置信息，并将这些信息存储在self.max_dis_num列表中
        if len(self.Left_obs) > 0:
            self.max_dis_num = []

            for i in range(0, int(len(self.Left_obs) / 2) + 1, 1):
                if i == 0:
                    # 从self.Left_obs[0] - 1开始，递减1，一直到0
                    for j in range(self.Left_obs[0] - 1, 0, -1):
                        if dis_90[j] <= dis_90[self.Left_obs[0] + 1]:
                            self.max_dis_num.extend([j, self.Left_obs[0] + 1])
                            break
                    # self.max_dis_num仍然为空
                    if not self.max_dis_num:
                        # 从self.Left_obs[0] - 1开始，递增1，一直到0
                        for j in range(0, self.Left_obs[0] - 1, 1):
                            if dis_90[j] >= dis_90[self.Left_obs[0] + 1]:
                                self.max_dis_num.extend([j, self.Left_obs[0] + 1])
                                break
                elif i < int(len(self.Left_obs) / 2):
                    # 将self.Left_obs列表中第2 * i - 1和第2 * i个元素直接添加到self.max_dis_num列表中
                    self.max_dis_num.extend([self.Left_obs[2 * i - 1], self.Left_obs[2 * i]])
                elif i == int(len(self.Left_obs) / 2):
                    # 从self.Left_obs[2 * i - 1] + 1开始，递增1，一直到lenth_dis - 1
                    for j in range(self.Left_obs[2 * i - 1] + 1, lenth_dis - 1, 1):
                        if dis_90[j] <= dis_90[self.Left_obs[2 * i - 1] - 1]:
                            self.max_dis_num.extend([self.Left_obs[2 * i - 1] - 1, j])
                            break
                    if len(self.max_dis_num) != 2 * i + 1:
                        # 从lenth_dis - 1开始，递减1，一直到self.Left_obs[2 * i - 1] + 1
                        for j in range(lenth_dis - 1, self.Left_obs[2 * i - 1] + 1, -1):
                            if dis_90[j] >= dis_90[self.Left_obs[2 * i - 1] - 1]:
                                self.max_dis_num.extend([self.Left_obs[2 * i - 1] - 1, j])
                                break
            # print("self.max_dis_num is", self.max_dis_num)
            
            #用来存储方向
            max_dis_val = 0

            #获取障碍物之间的最优方向
            for i in range(0, int(len(self.max_dis_num) / 2), 1):
                if max_dis_val < self.max_dis_num[2 * i + 1] - self.max_dis_num[2 * i]:
                    max_dis_val = self.max_dis_num[2 * i + 1] - self.max_dis_num[2 * i]
                    #取障碍物的中间序列
                    self.max_dis_index = (self.max_dis_num[2 * i + 1] + self.max_dis_num[2 * i]) / 2
                
                #根据条件对序列进行向右偏移
                if self.left == 1 and self.max_dis_index < 90 and dis_90[0] < dis_90[lenth_dis - 1] - 1:
                    self.max_dis_index = self.max_dis_index + 5 * abs(dis_90[lenth_dis - 1] - dis_90[0])
                #根据条件对序列进行向右偏移
                elif self.right == 1 and self.max_dis_index > 90 and dis_90[0] - 1 > dis_90[lenth_dis - 1]:
                    self.max_dis_index = self.max_dis_index - 5 * abs(dis_90[lenth_dis - 1] - dis_90[0])
     
     #判断条件并进行速度增幅               
    def speed_increase(self,dis_90_copy,speed):
        max_dis_list = []
        mean_max_dis=0
        if self.max_dir_index!=0 and len(self.Left_obs) ==0:
            for i in range(90-2,90+4,1):
                # dis_90_copy的数据没有经过限幅
                max_dis_list.append(dis_90_copy[i])
            #取距离的平均值
            mean_max_dis = np.mean(max_dis_list)
            #对速度进行增幅
            if mean_max_dis > self.MAX_DETECT_THRESHOLD and mean_max_dis < 5.5:
                incre = mean_max_dis / self.MAX_DETECT_THRESHOLD
                speed = (0.0910*pow(incre,2)-0.0604*pow(incre,1)+0.9697)*speed
            elif mean_max_dis >= 5.5   :#前面5.5米没有障碍
                speed = speed*1.1
    
    #消息发布
    def message_publish(self,steering_angle,speed):    
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle=steering_angle
        drive_msg.drive.speed=speed
        self.drive_pub.publish(drive_msg)
        
    def middle_line_callback(self,data):
        global last_angle
        # print(len(data.ranges))
        # print("+++++++++++++++++++")
        # print(data.angle_increment)
        dis = self.get_range(data, -90,90)# 前方180度雷达数据
        dis_90 = dis[::-1]
        lenth_dis = len(dis_90)
        dis_90_copy = tuple(dis_90)

        self.Left_obs_orig = []
        self.Left_obs = []
        
        #dis_90作为整体的激光数据进行限幅和替换零值操作
        dis_90 = self.find_max_dis(lenth_dis,dis_90)
        dis_90 = self.exchange(dis_90)

        
        # 从激光数据中找到连续的障碍物区间并滤波
        self.obstacle_intervals(lenth_dis,dis_90)
        # 计算所有障碍区间的方差，删除无效障碍
        self.obstacle_deviation(dis_90)
        # 障碍物膨胀
        self.obstacle_inflate(dis_90,lenth_dis)
        # 获取障碍物信息并计算最优方向
        self.direction_between_obstacles(dis_90,lenth_dis)


        #将达到最大检测值MAX_DETECT_THRESHOLD的激光序列成对存储起来
        self.max_dir_num = []
        for i in range(0, lenth_dis - 2):
            if dis_90[i] < self.MAX_DETECT_THRESHOLD and dis_90[i + 1] == self.MAX_DETECT_THRESHOLD and len(self.max_dir_num) % 2 == 0:
                self.max_dir_num.append(i + 1)
            elif dis_90[i] == self.MAX_DETECT_THRESHOLD and dis_90[i + 1] < self.MAX_DETECT_THRESHOLD and len(self.max_dir_num) % 2 == 1:
                self.max_dir_num.append(i)

        #计算最大检测值的平均方向和方向差，并且根据检测值的变化进行更新
        if len(self.max_dir_num) > 1:
            #两个激光序列的方向差是否大于4度
            if abs(self.max_dir_num[0] - self.max_dir_num[1]) > 4:
                #计算平均方向
                self.max_dir_index = (self.max_dir_num[0] + self.max_dir_num[1]) // 2
                #计算方向差
                self.max_dir = self.max_dir_num[1] - self.max_dir_num[0]
            # 与其他的最大检查方向进行比较
            if len(self.max_dir_num) > 2:
                for i in range(0, len(self.max_dir_num) // 2):
                    # 如果方向差更大的话
                    if self.max_dir_num[2 * i + 1] - self.max_dir_num[2 * i] > self.max_dir:
                        self.max_dir_index = (self.max_dir_num[2 * i + 1] + self.max_dir_num[2 * i]) // 2
                        self.max_dir = self.max_dir_num[2 * i + 1] - self.max_dir_num[2 * i]


        for i in range(0, len(self.Left_obs) - 1):
            #检查障碍物和最大检测方向之间的序列差是否小于20倍的1 / (dis_90[self.Left_obs[i]]和2之间的最小值
            if abs(self.max_dir_index - self.Left_obs[i]) < 20 * min(1 / (dis_90[self.Left_obs[i]] + 0.001), 2):
                self.max_dir_index = 0
                break

        dis_90[0] += 0.00001
        dis_90[lenth_dis - 1] += 0.00001
        angle_factor = 0.1 if dis_90[0] / dis_90[lenth_dis - 1] > 3 or dis_90[lenth_dis - 1] / dis_90[0] > 3 else 0.05

        # 计算角度angle
        if self.max_dir_index != 0:
            angle = -max(math.exp(-self.max_dis / self.MAX_DETECT_THRESHOLD), 0.7) * (self.max_dir_index - 90) / 360 * math.pi + angle_factor * (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])
        else:
            angle = -max(math.exp(-self.max_dis / self.MAX_DETECT_THRESHOLD), 0.7) * (self.max_dis_index - 90) / 360 * math.pi + angle_factor * (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])
            
        #加入PID并计算最终的角度和速度
        steering_angle = self.P* angle +self.D *(angle-last_angle)
        last_angle = angle
        speed = 2.5*(0.3*math.exp(-np.clip(abs(angle),0,0.5))+0.7)

        # 对速度进行增幅
        self.speed_increase(dis_90_copy,speed)
        #将速度和角度信息通过主题发布出去
        self.message_publish(steering_angle,speed)

if __name__ == '__main__': 
  try:
    rospy.init_node("run_in_sim")
    extendObj = DisparityExtenderDriving(decoupled=True)
    scan_sub = rospy.Subscriber('/scan', LaserScan, extendObj.middle_line_callback,queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
