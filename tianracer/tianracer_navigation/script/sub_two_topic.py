#!/usr/bin/env python2.7
#coding=utf-8
# created by Wu GengQian
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math
from std_msgs.msg import String
import message_filters
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

def callback(data1, data2):
    #rospy.loginfo( data1.angle_min,data2.orientation)
    #print("hello")
    print(data1.angle_min,data2.orientation)
    dis = get_range(data1, -90,90)# 前方180度雷达数据
    dis_90 = dis[::-1]
    #print(dis_90)
    theta = euler_from_quaternion([data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w])
    direct_origin = theta[2]*180/math.pi
    print(direct_origin)
    scan_msg =  LaserScan()
    scan_msg.angle_min = data1.angle_min
    scan_msg.angle_max = data1.angle_max
    scan_msg.angle_increment = data1.angle_increment
    scan_msg.time_increment = data1.time_increment
    scan_msg.scan_time = data1.scan_time
    scan_msg.range_min = data1.range_min
    scan_msg.range_max = data1.range_max
    scan_msg.ranges = dis_90
    scan_msg.intensities = data1.intensities
    scan_pub.publish(scan_msg)
def sub_two_topic():
    rospy.init_node('listener', anonymous=True)
    t1= message_filters.Subscriber("/scan", LaserScan)
    t2 =message_filters.Subscriber("/tianracer/imu",Imu)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 1, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
'''    drive_msg = AckermannDrive()
    drive_msg.steering_angle=steer_angle
    drive_msg.speed=speed
    drive_pub.publish(drive_msg)'''
    



if __name__ == '__main__': 
  try:
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
    scan_pub = rospy.Publisher('/scan2', LaserScan, queue_size=1)
    sub_two_topic()
    
  except rospy.ROSInterruptException:
    pass