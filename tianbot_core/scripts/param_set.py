#!/usr/bin/env python
import sys
import rospy
import time
from tianbot_core.srv import DebugCmd

def param_set_client(param):
    rospy.wait_for_service('/tianbot_core/debug_cmd_srv')
    count = 3
    while(1):
        try:
            debug_cmd = rospy.ServiceProxy('/tianbot_core/debug_cmd_srv', DebugCmd)
            resp = debug_cmd("param set " + param)
            return resp.result
        except rospy.ServiceException as e:
            count = count - 1
            if(count == 0):
                print("Service call failed: %s"%e)
                sys.exit(1)
            else:
                print("retry ...")

def param_get_client():
    rospy.wait_for_service('/tianbot_core/debug_cmd_srv')
    count = 3
    while(1):
        try:
            debug_cmd = rospy.ServiceProxy('/tianbot_core/debug_cmd_srv', DebugCmd)
            resp = debug_cmd("param get")
            return resp.result
        except rospy.ServiceException as e:
            count = count - 1
            if(count == 0):
                print("Service call failed: %s"%e)
                sys.exit(1)
            else:
                print("retry ...")

def param_save_client():
    rospy.wait_for_service('/tianbot_core/debug_cmd_srv')
    count = 3
    while(1):
        try:
            debug_cmd = rospy.ServiceProxy('/tianbot_core/debug_cmd_srv', DebugCmd)
            resp = debug_cmd("param save")
            return resp.result
        except rospy.ServiceException as e:
            count = count - 1
            if(count == 0):
                print("Service call failed: %s"%e)
                sys.exit(1)
            else:
                print("retry ...")

def reset_client():
    rospy.wait_for_service('/tianbot_core/debug_cmd_srv')
    try:
        debug_cmd = rospy.ServiceProxy('/tianbot_core/debug_cmd_srv', DebugCmd)
        resp = debug_cmd("reset")
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

def usage():
    return "%s [param_file_name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        filename = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print("param file [%s]\n"%filename)

    with open(filename, 'r' ) as file:
        for line in file.readlines():
            if line.strip() == '' or line.strip().startswith('#'):
                continue
            print("param set " + line.strip())
            print(param_set_client(line.strip()))

    print("param save ...")
    print(param_save_client())
    time.sleep(0.1)
    
    reset_client()
    print("wait 7 sec for reset ...\n")
    time.sleep(7)

    print("param get:")
    print(param_get_client())
