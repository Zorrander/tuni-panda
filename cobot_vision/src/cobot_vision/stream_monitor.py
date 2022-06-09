import tf
import abc
import cv2
import math
import rospy
import threading
import numpy as np
from cobot_msgs.msg import *
from cobot_msgs.srv import *
from sensor_msgs.msg import CameraInfo
from cobot_vision.camera_parameters import CameraParameters
from cobot_vision.poses_converter import PosesConverter

class ImageStreamMonitor(metaclass=abc.ABCMeta):
	
	def __init__(self, predictor, camera_params, camera_2_robot_pose_converter):
		self.predictor = predictor
		self.camera_params = camera_params
		self.camera_2_robot_pose_converter = camera_2_robot_pose_converter
		self.detection_pub = rospy.Publisher('/objects_detected', Detections, queue_size=10)
		self.robot_detection_pub = rospy.Publisher('/objects_detected_robot', Detections, queue_size=10)
		self.frame_number = 0

	@abc.abstractmethod
	def preprocess_img(self, img):
		pass

	@abc.abstractmethod
	def draw_outputs(self, rgb_image, detections):
		pass
	@abc.abstractmethod
	def process_detections(self, detections):
		pass

	@abc.abstractmethod
	def publish_results(self, detections):
		pass

	@abc.abstractmethod
	def store_detection(self, pred_class, x, y, quat):		
		pass

	@abc.abstractmethod
	def reset_detections(self):		
		pass

	def is_robot_connected(self):
		return self.camera_2_robot_pose_converter.is_robot_connected()

	def image_analyze(self, msg):
		msg = Detections() 
		response = GraspPoseDetectionResponse()
		for object_class in self.detections:
			for x,y, quat in self.detections[object_class]:
				detection = Detection()
				detection.obj_class = object_class
				detection.x = x
				detection.y = y
				detection.angle = quat
				msg.detections.append(detection)
		response.detection = msg
		return response

	def image_analyze_stream(self, rgb_image):
		# t = threading.currentThread()
		self.frame_number+=1

		if self.frame_number > 60 and self.frame_number%12==0:
			ready_rgb_image = self.preprocess_img(rgb_image)
			detections = self.predictor.predict(ready_rgb_image)
			processed_detections = self.process_detections(detections)
			if self.is_robot_connected():
				self.reset_detections()
				for detection in processed_detections:
					robot_pose = self.camera_2_robot_pose_converter.convert2Dpose(detection[2][0], detection[2][1])
					self.store_detection(detection[0], robot_pose, detection[3])
				self.publish_robot_object_poses()
			
			self.publish_results(processed_detections)
			self.draw_outputs(ready_rgb_image, processed_detections)


	def store_detection(self, pred_class, robot_pose, angle):
		pred_class = self.classes[pred_class]		
		if not self.detections[pred_class]:
			self.detections[pred_class] = [(robot_pose.position.x, robot_pose.position.y, angle)]
		else:
			if (not self.object_already_stored(pred_class, robot_pose.position.x, robot_pose.position.y)):
				#print("[STREAMER] append ({}, {})".format(x, y))
				self.detections[pred_class].append((robot_pose.position.x, robot_pose.position.y, angle)) 

	def object_already_stored(self, pred_class, x1, y1):
		distance_threshold = 0.01 # in m
		result = False 
		for x2,y2, quat in self.detections[pred_class]:
			if (abs(x1-x2) < distance_threshold and abs(y1-y2) < distance_threshold):
				result = True 
				break 
		return result

	def publish_robot_object_poses(self):
		msg = Detections()
		for object_class in self.detections:
			for x,y, quat in self.detections[object_class]:
				detection = Detection()
				detection.obj_class = object_class
				detection.x = x
				detection.y = y
				detection.angle = quat
				msg.detections.append(detection)
		self.robot_detection_pub.publish(msg)

