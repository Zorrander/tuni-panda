import tf
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
from cobot_vision.stream_monitor import ImageStreamMonitor

class DetectronStreamMonitor(ImageStreamMonitor):
	
	def __init__(self, predictor, camera_params, camera_2_robot_pose_converter):
		super().__init__(predictor, camera_params, camera_2_robot_pose_converter)
		self.detection_type = "bounding_box"

	def preprocess_img(self, ros_image):
		rgb_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(ros_image.height, ros_image.width, 3)
    	rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
		return rgb_image.copy()

	def predict(self, image):
		result = []
		for instance, pred_class, bounding_box, pred_angle, pred_kps_center in self.predictor.predict(image):
			result.append(pred_class, (bounding_box, pred_angle, pred_kps_center))
		return result


	def image_analyze(self, msg):
		print("image_analyze")
		print(self.detections)
		msg = Detections() 
		response = GraspPoseDetectionResponse()
		for object_class in self.detections:
			for x,y, quat in self.detections[object_class]:
				detection = Detection()
				detection.obj_class = object_class
				detection.x = x
				detection.y = y
				detection.quaternion = quat
				msg.detections.append(detection)
		response.detection = msg
		return response

	def image_analyze_stream(self, rgb_image):

		t = threading.currentThread()
		dt2_correction_flag=0

		analyze_img=rgb_image.copy()
		(winW, winH) = (224, 224)
		timer=0

		result = []
		msg = Detections()
		for instance, pred_class, bounding_box, pred_angle, pred_kps_center in self.predictor.predict(analyze_img):
		    detection = Detection()
		    bounding_box=np.asarray(bounding_box)
		    bounding_box=bounding_box.astype(int)
		    
		    if (instance > 0):
		        x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
		        ctr_X = int((bounding_box[0]+bounding_box[2])/2)
		        ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
		        # quat = self.pose_converter.convertAngle2Quaternion(pred_angle)

		        #ref_x = 640/2
		        #ref_y = 480/2
		        ref_x = self.camera_params.ctrX
		        ref_y = self.camera_params.ctrY
		        dist = [ctr_X - ref_x, ref_y - ctr_Y]
		        dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]

		        pose = self.camera_2_robot_pose_converter.convert2Dpose(dist[0], dist[1], pred_angle)
		        position = [pose.position.x, pose.position.y]
		        quat = [pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w]

		        # x_robot, y_robot = convert_detection_pose(dist[0], dist[1])
		        # msg.x = x_robot 
		        # msg.y = y_robot
		        detection.x = dist[0]
		        detection.y = dist[1]
		        detection.bounding_box = x1, y1, x2, y2
		        detection.kps_x = pred_kps_center[0]
		        detection.kps_y = pred_kps_center[1]
		        detection.quaternion = quat
		        detection.obj_class = pred_class
		        print(detection)
		        msg.detections.append(detection)
		        result.append([pred_class, bounding_box, pred_angle, pred_kps_center])

		        self.store_detection(pred_class, pose.position.x, pose.position.y, quat)
		        self.publish_robot_object_poses()
		if result: 
			self.detection_pub.publish(msg)
			self.draw_outputs(rgb_image, result)

		if len(self.detections[2]) == 2:
			obj1 = self.detections[2][0]
			obj2 = self.detections[2][1]
			#print("THERE ARE TWO GEARS")
			dist = math.sqrt((obj1[0] - obj2[0])**2 + (obj1[1] - obj2[1])**2)
			#print("dist(gear1 - gear2) = {}".format(dist))
			if (dist < 0.035):
				print("They are close enough, grasp them.")
		'''
		    return result
		else:
		    return [[9999,9999,9999,9999],9999,[9999,9999]]
		'''

	def store_detection(self, pred_class, x, y, quat):
				
		if not self.detections[pred_class]:
			self.detections[pred_class] = [(x, y, (quat))]
		else:
			if (not self.object_already_stored(pred_class, x, y)):
				#print("[STREAMER] append ({}, {})".format(x, y))
				self.detections[pred_class].append((x, y, (quat))) 

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
				detection.quaternion = quat
				msg.detections.append(detection)
		self.robot_detection_pub.publish(msg)

	def draw_outputs(self, myimage, outputs):
		for pred_class, bounding_box, pred_angle, pred_kps_center in outputs:
			ctr_X = int((bounding_box[0]+bounding_box[2])/2)
			ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
			center_coordinates = (int(ctr_X),int(ctr_Y))
			radius = 10
			if pred_class==4:
				color = (255, 0, 0)
			elif pred_class==1:
				color = (0, 255, 0)
			elif pred_class==2:
				color = (0, 0, 255)
			elif pred_class==3:
				color = (255, 255, 0)
			else:
				color = (0, 255, 255)
			thickness = 2
			clone = cv2.circle(myimage, center_coordinates, radius, color, thickness)
			out_img = cv2.rectangle(clone, (int(bounding_box[0]), int(bounding_box[1])), (int(bounding_box[2]), int(bounding_box[3])), color, 2)

		#cv2.putText(out_img, str(angle), (550,470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
		cv2.namedWindow("output", cv2.WINDOW_NORMAL)
		cv2.resizeWindow("output",1280,960)
		cv2.imshow("output", out_img)
		cv2.waitKey(1)