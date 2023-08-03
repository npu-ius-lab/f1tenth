#include "omni.h"
#include "protocol.h"

void TianbotOmni::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    struct twist twist;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, (uint8_t *)&twist, sizeof(twist));
    serial_.send(&buf[0], buf.size());

    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}

TianbotOmni::TianbotOmni(ros::NodeHandle *nh) : TianbotChasis(nh)
{
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TianbotOmni::velocityCallback, this);
}
