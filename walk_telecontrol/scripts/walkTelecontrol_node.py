#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import sys, tty, termios

import rospy
import rospkg

from std_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl

nodeControlId = 2
numberOfStep = 6
stepLength = [0.07, 0.04, 10.0]

gaitCommandPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=10)

def printTeleInfo():
    print '\n%-15s%s'%(' ', 'w--forward')
    print '%-15s%-15s%-15s'%('a--left', 's--backward', 'd--right')
    print '%-15s%-15s%-15s'%('z--trun left', 'x--in situ', 'c--trun right')
    print '%-15s%s\n'%(' ', 'q--quit')

def sendWalkCmd(deltax, deltay, theta):
    rospy.wait_for_message('/requestGaitCommand', Bool, 30)
    gaitCommandPub.publish(data=[deltax, deltay, theta])

def walkFewSteps(x,y,a,num):
    for i in range(0, num):
        sendWalkCmd(x, y, a)
        print '%s %-10s%-10s%-10s%-10s'%('step', i+1, x, y, a)
    printTeleInfo()

def getch(str=''):
    print str,
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print ch
    return ch

def keyboardControl():
    while not rospy.is_shutdown():
        cmd = getch('key:')
        if cmd == 'w':
            walkFewSteps(stepLength[0], 0.0, 0.0, numberOfStep)
        elif cmd == 'x':
            walkFewSteps(-stepLength[0], 0.0, 0.0, numberOfStep)
        elif cmd == 'a':
            walkFewSteps(0, stepLength[1], 0.0, numberOfStep)
        elif cmd == 'd':
            walkFewSteps(0, -stepLength[1], 0.0, numberOfStep)
        elif cmd == 'z':
            walkFewSteps(0.0, 0.0, stepLength[2], numberOfStep)
        elif cmd == 'c':
            walkFewSteps(0.0, 0.0, -stepLength[2], numberOfStep)
        elif cmd == 's':
            walkFewSteps(0.0, 0.0, 0.0, numberOfStep)
        elif cmd == 'q':
            return

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')

if __name__ == '__main__':
    rospy.init_node('walkTelecontrol_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)

    if mCtrl.SetBodyhubTo_walking(nodeControlId) == False:
        rospy.logerr('bodyhub to wlaking fail!')
        rospy.signal_shutdown('error')
        exit(1)
        
    printTeleInfo()
    keyboardControl()

    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('exit')

