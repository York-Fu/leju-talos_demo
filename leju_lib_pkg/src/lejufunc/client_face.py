#!/usr/bin/env python

import rospy
import json
import time

from ros_vision_node.srv import FaceRecognition, FaceVerify


def face_service(path=None):
	rospy.wait_for_service('ros_vision_node/face_detect', timeout=2)
	face_recognition = rospy.ServiceProxy('ros_vision_node/face_detect', FaceRecognition)

	if path is None:
		image_path = ""
	else:
		image_path = path

	response = face_recognition(image_path)
	if response.result == "Error":
		return []
	else:
		return json.loads(response.result)


def face_filter(face_list):
	target_face = face_list[0]
	target_face_size = target_face.get('faceRectangle').get('width') * target_face.get('faceRectangle').get('height')
	# print(target_face_size)

	if len(face_list) > 1:
		for face in face_list:
			current_face_size = face.get('faceRectangle').get('width') * face.get('faceRectangle').get('height')
			if current_face_size > target_face_size:
				target_face = face
				target_face_size = current_face_size

	return target_face


def age_check(age):
	if age > 0 and age <= 6:
		result = "child"
	elif age > 6 and age <= 17:
		result = "teen"
	elif age > 17 and age <= 40:
		result = "youth"
	elif age > 40 and age <= 65:
		result = "middle-age"
	else:
		result = "elder"

	return result


def face_age(age):
	"""
	Age check

	Args:
	  age: child, teen, youth, middle-age, elder

	Returns:
	  True / False for age check
	"""

	face_list = face_service()
	# print(face_list)
	if len(face_list) == 0:
		return False
	else:
		target_face = face_filter(face_list)
		# print(target_face)
		current_age = target_face.get('faceAttributes').get('age')
		if age == age_check(current_age):
			return True
		else:
			return False


def face_gender(gender):
	"""
	Gender check

	Args:
	  gender: male / female

	Returns:
	  True / False for gender check
	"""

	face_list = face_service()
	if len(face_list) == 0:
		return False
	else:
		target_face = face_filter(face_list)
		current_gender = target_face.get('faceAttributes').get('gender')
		if gender == current_gender:
			return True
		else:
			return False


def emotion_check(emotion_distribute):
	emotion_list = ["neutral", "sadness", "happiness", "disgust", "anger", "surprise", "fear", "contempt"]

	result = "neutral"
	emotion_rate = emotion_distribute.get("neutral")

	for cur_emotion in emotion_list:
		cur_emotion_rate = emotion_distribute.get(cur_emotion)
		if cur_emotion_rate > emotion_rate:
			emotion_rate = cur_emotion_rate
			result = cur_emotion

	return result


def face_emotion(emotion):
	"""
	Emotion check

	Args:
	  emotion: neutral, sadness, happiness, disgust, anger, surprise, fear, contempt

	Returns:
	  True / False for emotion check
	"""

	face_list = face_service()
	# print(face_list)
	if len(face_list) == 0:
		return False
	else:
		target_face = face_filter(face_list)
		emotion_distribute = target_face.get('faceAttributes').get('emotion')
		result = emotion_check(emotion_distribute)
		if result == emotion:
			return True
		else:
			return False


def face_detect(timeout, gender=None, age=None, emotion=None):
	"""
	Face in next several seconds

	Args:
	  timeout: time for detect face

	Returns:
	  True if face detected, False if no face detected
	"""

	start_time = time.time()
	face_list = []
	result = False

	while True:
		face_list = face_service()

		# print(face_list)
		if len(face_list) > 0:
			tmp_result = True
			target_face = face_filter(face_list)
			if tmp_result and gender is not None:
				current_gender = target_face.get('faceAttributes').get('gender')
				tmp_result = (current_gender == gender)
			if tmp_result and age is not None:
				current_age = age_check(target_face.get('faceAttributes').get('age'))
				tmp_result = (current_age == age)
			if tmp_result and emotion is not None:
				current_emotion = emotion_check(target_face.get('faceAttributes').get('emotion'))
				tmp_result = (current_emotion == emotion)

			result = tmp_result

		cur_time = time.time()
		if result or cur_time - start_time > timeout:
			break

	return result


def face_verify(file_name):
	"""
	Verify whether two faces belong to a same person

	Args:
	  filename: upload image file name

	Returns:
	  True if belongs to a same person, else false
	"""

	base_path = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/faces/'
	file_path = "%s%s" % (base_path, file_name)
	# print(file_path)

    # detect default face
	face_list = face_service(file_path)
	if len(face_list) == 0:
		return False
	else:
		target_face = face_filter(face_list)
		# print(target_face)
		faceId1 = target_face.get('faceId')

    # detect camera image
	face_list = face_service()
	if len(face_list) == 0:
		return False
	else:
		target_face = face_filter(face_list)
		# print(target_face)
		faceId2 = target_face.get('faceId')

	rospy.wait_for_service('ros_vision_node/face_verify', timeout=2)
	face_verify_client = rospy.ServiceProxy('ros_vision_node/face_verify', FaceVerify)
	res = face_verify_client(faceId1, faceId2)

	if res.result == 1:
		return True
	else:
		return False


def face_count():
	"""
	Return count of recognized faces
	"""

	face_list = face_service()
	# print(face_list)

	return len(face_list)


if __name__ == '__main__':
	rospy.init_node('face_test', anonymous=True)

	try:
		# result = face_age("teen")
		# if result:
		# 	print("You are teenager")
		# else:
		# 	print("You are not teenager")

		# result = face_gender("male")
		# if result:
		# 	print("You are male")
		# else:
		# 	print("You are female")

		# result = face_emotion("happiness")
		# if result:
		# 	print("You are happy")
		# else:
		# 	print("You are not happy")

		result = face_detect(10, "male", "youth", "neutral")
		print(result)
	except Exception as err:
		rospy.logerr(err)


