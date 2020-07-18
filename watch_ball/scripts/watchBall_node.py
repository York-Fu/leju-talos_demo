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

xPid = pidAlg.PositionPID(p=0.05,d=0.0)
yPid = pidAlg.PositionPID(p=0.04,d=0.0)

idList = [21,22]
valueList = [0,0]
zAxisLimit, yAxisLimit = 80, 30

# HSV阈值
lowerOrange = np.array([15, 100, 100])
upperOrange = np.array([25, 255, 255])
lowerRed = np.array([165, 128, 128])
upperRed = np.array([180, 224, 255])
lowerYellow = np.array([20, 192, 224])
upperYellow = np.array([30, 255, 255])

if SIMULATION:
    ball = imgPrcs.ColorObject(lowerOrange, upperOrange)
else:
    ball = imgPrcs.ColorObject(lowerRed, upperRed)

bridgeObj = CvBridge()
originImage = np.zeros((640,480,3), np.uint8)
fpsTime = 0

if SIMULATION:
    imageTopic = '/sim/camera/D435/colorImage'
else:
    imageTopic = '/camera/color/image_raw'

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

def watchBallLoop():
    global valueList
    result = ball.detection(originImage)
    if result['find'] != False:
        xError = originImage.shape[1]/2.0 - result['Cx']
        yError = originImage.shape[0]/2.0 - result['Cy']
        if (abs(xError) > 4) or (abs(yError) > 4):
            valueList[0] = valueList[0] + xPid.run(xError)
            valueList[1] = valueList[1] + (-yPid.run(yError))
            # output limiting
            valueList[0] = valueList[0] > zAxisLimit and zAxisLimit or valueList[0]
            valueList[0] = valueList[0] < -zAxisLimit and -zAxisLimit or valueList[0]
            valueList[1] = valueList[1] > yAxisLimit and yAxisLimit or valueList[1]
            valueList[1] = valueList[1] < -yAxisLimit and -yAxisLimit or valueList[1]
            mCtrl.SendJointCommand(nodeControlId, idList, valueList)

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')

if __name__ == '__main__':
    rospy.init_node('watchBall_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber(imageTopic, Image, imageCallback)

    rospy.loginfo('SIMULATION: %s',SIMULATION) 
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_setStatus(nodeControlId) == False:
        rospy.logerr('bodyhub to setStatus fail!')
        rospy.signal_shutdown('error')
        exit(1)

    loopRate = rospy.Rate(10)
    while not rospy.is_shutdown():
        watchBallLoop()
        loopRate.sleep()
    