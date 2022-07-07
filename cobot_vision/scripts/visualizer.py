#!/usr/bin/env python3
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cobot_msgs.msg import Detection

class DetectionVisualizer(object):

    def __init__(self):
        self.bounding_box = [-1, -1, -1, -1]
        self.pred_angle = -1
        self.pred_kps_center = [-1, -1]

    def detection_callback(self, detection_msg):
        print(detection_msg)
        self.bounding_box = detection_msg.bounding_box
        self.pred_angle = detection_msg.angle
        self.pred_kps_center = [detection_msg.kps_x, detection_msg.kps_y]

    def img_callback(self, image_msg):
        self.rgb_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width,3)
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        self.draw_outputs()

    def draw_outputs(self):
        center_coordinates = (int(self.pred_kps_center[0]),int(self.pred_kps_center[1]))

        radius = 10
        color = (255, 0, 0)
        thickness = 2
        clone = cv2.circle(self.rgb_image, center_coordinates, radius, color, thickness)
        out_img = cv2.rectangle(clone, (int(self.bounding_box[0]), int(self.bounding_box[1])), (int(self.bounding_box[2]), int(self.bounding_box[3])), (255,0,0), 2)

        cv2.putText(out_img, str(self.pred_angle), (550,470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output",1280,960)
        cv2.imshow("output", out_img)
        cv2.waitKey(1)
        self.bounding_box = [-1, -1, -1, -1]
        self.pred_angle = -1
        self.pred_kps_center = [-1, -1]
        
if __name__ == '__main__':
    rospy.init_node('detection_viz', anonymous=True)
    visualizer = DetectionVisualizer()
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, visualizer.img_callback)
    detection_sub = rospy.Subscriber("objects_detected", Detection, visualizer.detection_callback)
    rospy.spin()
