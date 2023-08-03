#ifndef __DIFFERENTIAL_H__
#define __DIFFERENTIAL_H__

#include "ros/ros.h"
#include "chassis.h"
#include "geometry_msgs/Twist.h"

class TianbotDifferential : public TianbotChasis{
public:
    TianbotDifferential(ros::NodeHandle *nh);
private:
    ros::Subscriber cmd_vel_sub_;
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

#endif
