#ifndef __OMNI_H__
#define __OMNI_H__

#include "ros/ros.h"
#include "chassis.h"
#include "geometry_msgs/Twist.h"

class TianbotOmni : public TianbotChasis{
public:
    TianbotOmni(ros::NodeHandle *nh);
private:
    ros::Subscriber cmd_vel_sub_;
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

#endif
