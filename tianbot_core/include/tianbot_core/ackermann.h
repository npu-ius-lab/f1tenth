#ifndef __ACKERMANN_H__
#define __ACKERMANN_H__

#include "ros/ros.h"
#include "chassis.h"
#include "ackermann_msgs/AckermannDrive.h"

class TianbotAckermann : public TianbotChasis
{
public:
    TianbotAckermann(ros::NodeHandle *nh);

private:
    ros::Subscriber ackermann_sub_;
    void ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr &msg);
};

#endif
