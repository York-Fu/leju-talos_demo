#!/usr/bin/env python

import os
import sys

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from gesture_detec_demo.msg import Gesture
from inf_gesture import *

ModelPath = os.path.abspath(os.path.join(os.path.dirname(__file__), "./model"))
Detector = gDetector(os.path.join(ModelPath, 'gesture'), os.path.join(ModelPath, 'gesture.cfg'), os.path.join(ModelPath, 'gesture.names'))
Bridge = CvBridge()
GesturePub = rospy.Publisher("/camera/gesture", Gesture, queue_size=1)


def GestureProcess(image):
  t0 = time.time()
  boxes, indices, classes, class_ids = Detector.detect(image)

  # Draw detections
  for i in indices:
    ind = i[0]
    box = boxes[ind]
    x = box[0]
    y = box[1]
    w = box[2]
    h = box[3]
    cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 2)
    cv2.putText(image, classes[class_ids[i[0]]], (int(x - 10), int(y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

  fps = round(1 / (time.time() - t0), 3)
  cv2.putText(image, 'fps:' + str(fps), (int(10), int(20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
  cv2.imshow('gesture', image)
  cv2.waitKey(1)

  gestureMsg = Gesture()
  gestureMsg.lableList = classes
  gestureMsg.indexList = class_ids
  GesturePub.publish(gestureMsg)


def ColorCallback(data):
  try:
    cv_image = Bridge.imgmsg_to_cv2(data, "bgr8")
    GestureProcess(cv_image)

  except CvBridgeError as e:
    print(e)


def main(args):
  rospy.init_node('GestureDetecDemo', anonymous=True)
  GestureSub = rospy.Subscriber("/camera/color/image_raw", Image, ColorCallback)
  rospy.loginfo("Start 'GestureDetecDemo' node...")
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
