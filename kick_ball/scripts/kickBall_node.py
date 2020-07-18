#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import sys, tty, termios

import rospy
import rospkg

import cv2
import numpy as np 
from cv_bridge import *
from sensor_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl

sys.path.append(rospkg.RosPack().get_path('publiclib_pkg'))
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg

SIMULATION = False
nodeControlId = 2

if SIMULATION:
    setpLength = [0.12, 0.08, 10.0] # x, y ,theta
    errorThreshold = [10.0, 10.0, 10.0]
    
    xPid = pidAlg.PositionPID(p=0.0016)
    yPid = pidAlg.PositionPID(p=0.001)
    aPid = pidAlg.PositionPID(p=0.18)
else:
    setpLength = [0.06, 0.03, 10.0] # x, y ,theta
    errorThreshold = [20.0, 20.0, 20.0]

    xPid = pidAlg.PositionPID(p=0.001)
    yPid = pidAlg.PositionPID(p=0.0003)
    aPid = pidAlg.PositionPID(p=0.09)

# HSV阈值
lowerOrange = np.array([15, 100, 100])
upperOrange = np.array([25, 255, 255])
lowerCyan = np.array([80, 100, 100])
upperCyan = np.array([95, 255, 255])
lowerRed = np.array([0, 224, 96])
upperRed = np.array([16, 255, 240])

if SIMULATION:
    ball = imgPrcs.ColorObject(lowerOrange, upperOrange)
else:
    ball = imgPrcs.ColorObject(lowerRed, upperRed)

hole = imgPrcs.ColorObject(lowerCyan, upperCyan)

bridgeObj = CvBridge()
originImage = np.zeros((640,480,3), np.uint8)
fpsTime = 0

if SIMULATION:
    imageTopic = '/sim/camera/UVC/colorImage'
else:
    imageTopic = '/chin_camera/image'

def imageCallback(msg):
    global originImage, fpsTime
    try:
        originImage = bridgeObj.imgmsg_to_cv2(msg, 'bgr8')
        
    except CvBridgeError as err:
        rospy.logerr(err)

    if False:
        t0 = time.time()
        imgPrcs.putVisualization(originImage, ball.detection(originImage))
        t1 = time.time()
        fps = 1.0/(time.time() - fpsTime)
        fpsTime = time.time()
        imgPrcs.putTextInfo(originImage,fps,(t1-t0)*1000)
        cv2.imshow("Image window", originImage)
        cv2.waitKey(1)

def GoToBall(targetx,targety,targeta):
    global errorThreshold
    while not rospy.is_shutdown():
        result = ball.detection(originImage)
        if result['find'] != False:
            xError = targetx - result['Cy']
            yError = targety - result['Cx']
            aError = targeta - result['Cx']
            if (abs(xError) < errorThreshold[0]) and (abs(yError) < errorThreshold[1]) and (abs(aError) < errorThreshold[2]):
                break
            xLength = xPid.run(xError)
            yLength = yPid.run(yError)
            aLength = aPid.run(aError)
            mCtrl.WalkTheDistance(xLength, yLength, aLength)
            mCtrl.WaitForWalkingDone()
        else:
            rospy.logwarn('no ball found!')
            time.sleep(0.5)

def SimPrepareKickBall(targetx,targety,targeta):
    global errorThreshold
    while not rospy.is_shutdown():
        result1 = ball.detection(originImage)
        result2 = hole.detection(originImage)
        if (result1['find'] != False) and (result2['find'] != False):
            xError = targetx - result1['Cy']
            yError = targety - result1['Cx']
            aError = targeta - result2['Cx']
            if (abs(xError) < errorThreshold[0]) and (abs(yError) < errorThreshold[1]) and (abs(aError) < errorThreshold[2]):
                break
            xLength = xPid.run(xError)
            yLength = yPid.run(yError)
            aLength = aPid.run(aError)
            mCtrl.WalkTheDistance(xLength, yLength, aLength)
            mCtrl.WaitForWalkingDone()

def SimkickBall():
    global setpLength
    GoToBall(330.0,320.0,320.0)
    SimPrepareKickBall(360.0,280.0,285.0)
    mCtrl.SendGaitCommand(setpLength[0], 0.0, 0.0)
    mCtrl.WaitForWalkingDone()

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')


if __name__ == '__main__':
    rospy.init_node('kickBall_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber(imageTopic, Image, imageCallback)

    rospy.loginfo('SIMULATION: %s',SIMULATION) 
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_walking(nodeControlId) == False:
        rospy.logerr('bodyhub to wlaking fail!')
        rospy.signal_shutdown('error')
        exit(1)
    time.sleep(2)
    while not rospy.is_shutdown():
        if SIMULATION:
            SimkickBall()
            mCtrl.ResetBodyhub()
            rospy.signal_shutdown('exit')
        else:
            GoToBall(240.0,260.0,260.0)
            mCtrl.SendGaitCommand(0.06, 0.0, 0.0)
            mCtrl.WaitForWalkingDone()
            mCtrl.SendGaitCommand(0.12, 0.0, 0.0)
            mCtrl.WaitForWalkingDone()

