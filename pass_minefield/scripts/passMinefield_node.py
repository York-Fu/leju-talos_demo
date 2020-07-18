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

nodeControlId = 2

setpLength = [0.1, 0.06, 10.0] # x, y ,theta
errorThreshold = [4.0, 2.0, 2.0]

xPid = pidAlg.PositionPID(p=0.0014)
yPid = pidAlg.PositionPID(p=0.00115)

# HSV阈值
lowerBlack = np.array([0, 0, 0])
upperBlack = np.array([2, 2, 10])
lowerBlue = np.array([110, 192, 192])
upperBlue = np.array([130, 255, 255])

mine = imgPrcs.ColorObject(lowerBlack, upperBlack)
wall = imgPrcs.ColorObject(lowerBlue, upperBlue)

bridgeObj = CvBridge()
originImage = np.zeros((640,480,3), np.uint8)
fpsTime = 0
roiW, roiH = 240, 200

imageTopic = '/sim/camera/UVC/colorImage'

def imageCallback(msg):
    global originImage, fpsTime
    try:
        originImage = bridgeObj.imgmsg_to_cv2(msg, 'bgr8')
        w, h = originImage.shape[1], originImage.shape[0]
        originImage = originImage[h-roiH:h-40, w/2-roiW/2:w/2+roiW/2]
    except CvBridgeError as err:
        rospy.logerr(err)

    if False:
        t0 = time.time()
        imgPrcs.putVisualization(originImage, mine.detection(originImage))
        imgPrcs.putVisualization(originImage, wall.detection(originImage))
        t1 = time.time()
        fps = 1.0/(time.time() - fpsTime)
        fpsTime = time.time()
        imgPrcs.putTextInfo(originImage,fps,(t1-t0)*1000)
        cv2.imshow("Image window", originImage)
        cv2.waitKey(1)

def GoToXOfMine(targetx):
    global errorThreshold
    while not rospy.is_shutdown():
        result = mine.detection(originImage)
        if result['find'] == True:
            xError = targetx - result['Cy']
            if abs(xError) < errorThreshold[0]:
                break
            xLength = xPid.run(xError)
            mCtrl.WalkTheDistance(xLength, 0, 0)
            mCtrl.WaitForWalkingDone()

def GoToYOfMine(targety):
    global errorThreshold
    while not rospy.is_shutdown():
        result = mine.detection(originImage)
        if result['find'] == True:
            w, h = originImage.shape[1], originImage.shape[0]
            if result['Cx'] > w/2:
                yError = (w-targety) - result['Cx']
            else:
                yError = targety - result['Cx']
            if abs(yError) < errorThreshold[0]:
                break
            yLength = yPid.run(yError)
            mCtrl.WalkTheDistance(0, yLength, 0)
            mCtrl.WaitForWalkingDone()

def passMinefield():
    while not rospy.is_shutdown():
        result = wall.detection(originImage)
        if result['find'] == True:
            break
        result = mine.detection(originImage)
        if result['find'] == True:
            GoToXOfMine(115)
            GoToYOfMine(8)
        for i in range(0, 3):
            mCtrl.SendGaitCommand(0.04, 0.0, 0.0)
        mCtrl.WaitForWalkingDone()

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')

if __name__ == '__main__':
    rospy.init_node('passMinefield_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber(imageTopic, Image, imageCallback)
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_walking(nodeControlId) == False:
        rospy.logerr('bodyhub to wlaking fail!')
        rospy.signal_shutdown('error')
        exit(1)
    time.sleep(1)
    
    passMinefield()

    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('exit')
     