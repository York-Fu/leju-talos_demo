#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import array
from ServoJointTrajectory import ServoJointTrajectoryC, FRAME_TIME
from bodyhub.srv import SrvServoAllRead
import time
import os
import subprocess
import Queue
import threading
from bezier import generator_bezier
from client_logger import serror
from utils import judge_range_number 
import sys

DEFAULT_TIME = 100

# TODO: immature code inside
SERVO = ServoJointTrajectoryC()
audio_path = os.environ['HOME'] + '/Music/actmusic/'


class PlayAudio(threading.Thread):
    def __init__(self,music_queue):
        threading.Thread.__init__(self)
        self._queue = music_queue
        self.proc = None
        self.playing = False

    def run(self):
        while not self._queue.empty():
            self.playing = True
            music_file,delay_time = self._queue.get(block=False)
            file_path = '%s%s' % (audio_path, music_file)
            self.proc = subprocess.call(['play', file_path])
            time.sleep(delay_time / 1000.0)
        self.playing = False

def music_action(music, act_frames):
    """

    :param music: music_list
    :param act_frames: frame_list
    :return:
    """
    music_queue = Queue.Queue()
    for elem in music:
        music_queue.put(elem)
    playaudio = PlayAudio(music_queue)
    playaudio.start()

    action(act_frames)

    while playaudio.playing:
        time.sleep(0.01)

def _list_to_array(act_list):
    return array.array("d", act_list)


def set_all_servo(p1, act_time=10, p0=None, p2=None):
    SERVO.MotoJointTransfer(p1, act_time, p0, p2)
    SERVO.MotoWait()


def action(act_frames):
    """
    Execute action

    Args:
      act_frames: a frame list
    """
    p0 = SERVO.GetMotoValue()
    act_frames.insert(0, (p0, 0, 0))
    for frame, sleep_time, is_last in generator_bezier(act_frames):
        SERVO.MotoJointFrames(frame)
        SERVO.MotoWait(True if is_last else sleep_time > 0)
        time.sleep(sleep_time)

def get_servo_value(id=None):
    """
    Get all servo value

    Args:
      id: servo id, 1 ~ 20

    Returns:
      value of servo
    """

    id_array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    id_count = 20

    rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
    servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
    result = servo_client(id_array, id_count)
    servo_value = result.getData
    # print(servo_value)

    if id is None:
        return servo_value
    else:
        return servo_value[id-1]


def set_servo_value(id, value, act_time=DEFAULT_TIME):
    """
    Set servo value

    Args:
      id: servo id, 1 ~ 20
      value: servo value
      act_time: action duration, use default value
    """
    try:
        id = int(judge_range_number(id, 1, 22))
    except Exception as err:
        lineno = sys._getframe().f_back.f_lineno
        co_name = sys._getframe().f_code.co_name
        serror("舵机编号的{}, line {}, in {}".format(err.message, lineno, co_name))
        return

    try:
        value = int(judge_range_number(value, -150, 150))
    except Exception as err:
        lineno = sys._getframe().f_back.f_lineno
        co_name = sys._getframe().f_code.co_name
        serror("舵机角度的{}, line {}, in {}".format(err.message, lineno, co_name))
        return
    try:
        act_time = act_time / 1000
        act_time = int(judge_range_number(act_time, 0.02, 180))
        act_time = act_time * 1000
    except Exception as err:
        lineno = sys._getframe().f_back.f_lineno
        co_name = sys._getframe().f_code.co_name
        serror("运行时间的{}, line {}, in {}".format(err.message, lineno, co_name))
        return

    current_servo = get_servo_value()
    new_servo = []
    for servo in current_servo:
        new_servo.append(servo)

    new_servo[id-1] = value
    set_all_servo(new_servo, act_time)


if __name__ == '__main__':
    rospy.init_node("action_test", anonymous=True)
    rospy.sleep(2.0)

    result = get_servo_value(5)
    print(result)
