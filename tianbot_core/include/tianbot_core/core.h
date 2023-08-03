#ifndef __CORE_H__
#define __CORE_H__

#include "ros/ros.h"
#include "serial.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "string.h"
#include "std_msgs/String.h"
#include "tianbot_core/DebugCmd.h"
#include <string>
#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUDRATE 460800
#define DEFAULT_TYPE "omni"
#define DEFAULT_TYPE_VERIFY true

using namespace std;
using namespace boost;

class TianbotCore
{
public:
    Serial serial_;
    ros::Publisher debug_result_pub_;
    ros::Subscriber debug_cmd_sub_;
    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;

    bool debugResultFlag_;
    string debugResultStr_;

    TianbotCore(ros::NodeHandle *nh);
    void checkDevType(void);
    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;

private:

    ros::Timer communication_timer_;

    void serialDataProc(uint8_t *data, unsigned int data_len);
    void heartCallback(const ros::TimerEvent &);
    void communicationErrorCallback(const ros::TimerEvent &);
    void debugCmdCallback(const std_msgs::String::ConstPtr &msg);
    bool debugCmdSrv(tianbot_core::DebugCmd::Request &req,  tianbot_core::DebugCmd::Response &res);
    ros::ServiceServer param_set_;
};

#endif
