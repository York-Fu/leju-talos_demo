#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import *
from sensor_msgs.msg import *

from bodyhub.msg import *
from bodyhub.srv import *

walkingState = 0

jointControlPub = rospy.Publisher('MediumSize/BodyHub/jointControl', JointState, queue_size=1000)
jointPositionPub = rospy.Publisher('MediumSize/BodyHub/MotoPosition', JointControlPoint, queue_size=1000)
gaitCommandPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=10)

def requestGaitCallback(msg):
    pass

def walkingStateCallback(data):
    global walkingState
    walkingState = data.data

rospy.Subscriber('/requestGaitCommand', Bool, requestGaitCallback)
rospy.Subscriber("/MediumSize/BodyHub/WalkingStatus",Float64, walkingStateCallback, queue_size=10)

def GetBodyhubStatus():
    rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 30)
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
    response = client('get')
    return response

def SetBodyhubStatus(id, status):
    rospy.wait_for_service('MediumSize/BodyHub/StateJump', 30)
    client = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
    response = client(id, status)
    return response.stateRes

def GetbodyhubControlId():
    rospy.wait_for_service('MediumSize/BodyHub/GetMasterID', 30)
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
    response = client('get')
    return response.data

def GetJointAngle(jointIdList):
    rospy.wait_for_service('MediumSize/BodyHub/GetJointAngle', 30)
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
    response = client(jointIdList, 22)
    return response.getData

def SetBodyhubTo_setStatus(concrolId):
    status = GetBodyhubStatus() 
    if status.data != 'preReady':
        id = GetbodyhubControlId()
        if (id != 0) and (id != concrolId):
            rospy.logwarn('bodyhub busy, id is %d'%(id))
            return False
        if (id == concrolId) and (status.data == 'ready'):
            return True
        else:
            result = SetBodyhubStatus(id, 'stop')
            if result != 25:
                return False
            result = SetBodyhubStatus(concrolId, 'reset')
            if result != 21:
                return False

    result = SetBodyhubStatus(concrolId, 'setStatus')
    if result != 22:
        return False
    return True

def SetBodyhubTo_walking(concrolId):
    status = GetBodyhubStatus() 
    if status.data == 'preReady':
        result = SetBodyhubStatus(concrolId, 'setStatus')
        if result != 22:
            return False
    else:
        id = GetbodyhubControlId()
        if (id != 0) and (id != concrolId):
            rospy.logwarn('bodyhub busy, id is %d'%(id))
            return False
        if (id == concrolId) and (status.data == 'walking'):
            return True
        
    result = SetBodyhubStatus(concrolId, 'walking')
    if result != 28:
        return False
    return True

def ResetBodyhub():
    status = GetBodyhubStatus() 
    if status.data != 'preReady':
        id = GetbodyhubControlId()
        result = SetBodyhubStatus(id, 'stop')
        if result != 25:
            return False
        result = SetBodyhubStatus(id, 'reset')
        if result != 21:
            return False
    return True

def SendJointCommand(controlId, idList, posList):
    strList = [str(i) for i in idList]
    strList.append(str(controlId))
    jointControlPub.publish(name=strList, position=posList)

def SendJointPsition(controlId, posList):
    jointPositionPub.publish(positions=posList, mainControlID=controlId)

def WaitForActionExecute(completely=False):
    status = GetBodyhubStatus() 
    if completely:
        while status.data != 'pause':
            status = GetBodyhubStatus()
    else:
        while status.poseQueueSize > 50:
            status = GetBodyhubStatus()

def SendGaitCommand(deltax, deltay, theta):
    global walkingState
    rospy.wait_for_message('/requestGaitCommand', Bool, 30)
    walkingState = 1
    gaitCommandPub.publish(data=[deltax, deltay, theta])

def WaitForWalkingDone():
    global walkingState
    while (walkingState != 0) and (not rospy.is_shutdown()):
        pass

def WalkTheDistance(xLength,yLength,aLength):
    setpLength = [0.06, 0.03, 10.0] # x, y ,theta

    direction = [0,0,0]
    if xLength >= 0:
        direction[0] = 1
    else:
        direction[0] = -1
    if yLength >= 0:
        direction[1] = 1
    else:
        direction[1] = -1
    if aLength >= 0:
        direction[2] = 1
    else:
        direction[2] = -1

    xNumberOfStep = abs(xLength)//setpLength[0]
    yNumberOfStep = abs(yLength)//setpLength[1]
    aNumberOfStep = abs(aLength)//setpLength[2]
    xLength = xLength - ((setpLength[0] * xNumberOfStep)*direction[0])
    yLength = yLength - ((setpLength[1] * yNumberOfStep)*direction[1])
    aLength = aLength - ((setpLength[2] * aNumberOfStep)*direction[2])
    numberOfStep = max(xNumberOfStep, yNumberOfStep, aNumberOfStep)

    for i in range(0, int(numberOfStep+1)):
        if xNumberOfStep >=1:
            x = setpLength[0]*direction[0]
            xNumberOfStep = xNumberOfStep - 1
        elif xNumberOfStep == 0:
            x = xLength
            xNumberOfStep = xNumberOfStep - 1
        else:
            x = 0.0

        if yNumberOfStep >=1:
            y = setpLength[1]*direction[1]
            yNumberOfStep = yNumberOfStep - 1
        elif yNumberOfStep == 0:
            y = yLength
            yNumberOfStep = yNumberOfStep - 1
        else:
            y = 0.0

        if aNumberOfStep >=1:
            a = setpLength[2]*direction[2]
            aNumberOfStep = aNumberOfStep - 1
        elif aNumberOfStep == 0:
            a = aLength
            aNumberOfStep = aNumberOfStep - 1
        else:
            a = 0.0
        SendGaitCommand(x, y, a)
    WaitForWalkingDone()
