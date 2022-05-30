import tf
import cv2
import rospy
import threading
import numpy as np
from cobot_msgs.msg import *
from cobot_msgs.srv import *
from sensor_msgs.msg import CameraInfo
from cobot_vision.camera_parameters import CameraParameters
from cobot_vision.poses_converter import PosesConverter

class ImageStreamMonitor():
	
	def __init__(self, object_detector):
		self.object_detector = object_detector
		self.detection_pub = rospy.Publisher('/objects_detected', Detections, queue_size=10)
		self.robot_detection_pub = rospy.Publisher('/objects_detected_robot', Detections, queue_size=10)
		# rospy.Timer(rospy.Duration(2), self.reset_detection_callback)

		self.listener_tf = tf.TransformListener()

		self.camera_params = CameraParameters()
		rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_params.camera_info_callback)

		self.pose_converter = PosesConverter(self.listener_tf, self.camera_params, "/panda_link0", "/camera_color_frame")

		self.detections = {
			1: [],
			2: [],
			3: [],
			4: [],
			5: [],
		}

	def reset_detection_callback(self, event):
	    print('Timer called at ' + str(event.current_real))
	    self.detections = []

	def image_analyze(self, msg):
		msg = Detections() 
		response = GraspPoseDetectionResponse()
		for object_class in self.detections:
			for x,y in self.detections[object_class]:
				detection = Detection()
				detection.obj_class = object_class
				detection.x = x
				detection.y = y
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
	    for instance, pred_class, bounding_box, pred_angle, pred_kps_center in self.object_detector.predict(analyze_img):
	        detection = Detection()
	        bounding_box=np.asarray(bounding_box)
	        bounding_box=bounding_box.astype(int)
	        
	        if (instance > 0):
	            x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
	            ctr_X = int((bounding_box[0]+bounding_box[2])/2)
	            ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
	            angle = pred_angle
	            #ref_x = 640/2
	            #ref_y = 480/2
	            ref_x = self.camera_params.ctrX
	            ref_y = self.camera_params.ctrY
	            dist = [ctr_X - ref_x, ref_y - ctr_Y]
	            dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
	            # x_robot, y_robot = convert_detection_pose(dist[0], dist[1])
	            # msg.x = x_robot 
	            # msg.y = y_robot
	            detection.x = dist[0]
	            detection.y = dist[1]
	            detection.bounding_box = x1, y1, x2, y2
	            detection.kps_x = pred_kps_center[0]
	            detection.kps_y = pred_kps_center[1]
	            detection.angle = pred_angle
	            detection.obj_class = pred_class

	            msg.detections.append(detection)
	            result.append([pred_class, bounding_box, pred_angle, pred_kps_center])

	            self.store_detection(pred_class, dist[0], dist[1])
	            self.publish_robot_object_poses()
	    if result: 
	    	self.detection_pub.publish(msg)
	    	self.draw_outputs(rgb_image, result)
	    '''
	        return result
	    else:
	        return [[9999,9999,9999,9999],9999,[9999,9999]]
		'''

	def store_detection(self, pred_class, x, y):
		pose = self.pose_converter.convert2Dpose(x, y)
		if not self.detections[pred_class]:
			self.detections[pred_class] = [(pose.position.x, pose.position.y)]
		else:
			if (not self.object_already_stored(pred_class, pose.position.x, pose.position.y)):
				print("[STREAMER] append ({}, {})".format(x, y))
				self.detections[pred_class].append((pose.position.x, pose.position.y)) 

	def object_already_stored(self, pred_class, x1, y1):
		distance_threshold = 0.01 # in m
		result = False 
		for x2,y2 in self.detections[pred_class]:
			if (abs(x1-x2) < distance_threshold and abs(y1-y2) < distance_threshold):
				result = True 
				break 
		return result

	def publish_robot_object_poses(self):
		msg = Detections()
		for object_class in self.detections:
			for x,y in self.detections[object_class]:
				detection = Detection()
				detection.obj_class = object_class
				detection.x = x
				detection.y = y
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