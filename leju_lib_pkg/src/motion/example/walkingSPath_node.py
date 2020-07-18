#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import sys, tty, termios

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl

nodeControlId = 2

def walkingSPath():
    for i in range(0, 5):
        mCtrl.SendGaitCommand(0.0, 0.0, 12.0)
    for i in range(0, 16):
        mCtrl.SendGaitCommand(0.07, 0.0, -7.5)
    for i in range(0, 16):
        mCtrl.SendGaitCommand(0.07, 0.0, 7.5)
    for i in range(0, 5):
        mCtrl.SendGaitCommand(0.0, 0.0, -12.0)

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')

if __name__ == '__main__':
    rospy.init_node('walking_SPath_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_walking(nodeControlId) == False:
        rospy.logerr('bodyhub to walking fail!')
        rospy.signal_shutdown('error')
        exit(1)
    time.sleep(1)

    while not rospy.is_shutdown():
        walkingSPath()
        mCtrl.WaitForWalkingDone()
        mCtrl.ResetBodyhub()
        rospy.signal_shutdown('exit')

