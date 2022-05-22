#!/usr/bin/env python3

import sys
import copy
from detectron2_inference import detectron_model, drawRect, rot, get_straight_bbox
sys.path.append('/home/mehmanse/catkin_build_ws/install/lib/python3/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python3/dist-packages')
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import threading
import time
from std_msgs.msg import Int16
import threading
from numpy import unravel_index
from std_msgs.msg import Float32MultiArray
import pandas as pd
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import rospy
import roslib
import numpy as np
from cobot_msgs.srv import GraspPoseDetection, GraspPoseDetectionResponse
from cobot_msgs.msg import Detection, Detections
import tf

CUDA_LAUNCH_BLOCKING=1

def callback(image_data):
    global rgb_image
    global object_num
    # rgb_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="rgb8")
    rgb_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width,3)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

    outputs = image_analyze_stream(object_num)
    draw_outputs(rgb_image, outputs)
    #bounding_box, pred_angle, pred_kps_center = image_analyze_stream(object_num)
    #draw_outputs(rgb_image, bounding_box, pred_angle, pred_kps_center)



'''
def request_callback(data):
    global object_num
    print ("new request:")
    print (data.data)
    object_num = data.data
    image_analyze(object_num)
'''



def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def crop_rect(dims, crop_scale):

    dimensions=dims.copy()
    xy_dims=[abs(dimensions[2]-dimensions[0]), abs(dimensions[3]-dimensions[1])]

    max_edge = max(xy_dims)
    if crop_scale>=1:
        to_correct = int(max_edge*(crop_scale - 1)/2)
        dimensions[0] -= to_correct
        dimensions[1] -= to_correct
        dimensions[2] += to_correct
        dimensions[3] += to_correct
    else:
        to_correct = int(max_edge*(1 - crop_scale)/2)
        dimensions[0] += to_correct
        dimensions[1] += to_correct
        dimensions[2] -= to_correct
        dimensions[3] -= to_correct

    xy_dims=[abs(dimensions[2]-dimensions[0]), abs(dimensions[3]-dimensions[1])]
    xy_idx = np.argmax(xy_dims)
    half_dist=int(abs(xy_dims[1]-xy_dims[0])/2)

    New_x1=dimensions[0]-half_dist
    New_y1=dimensions[1]-half_dist
    New_x2=dimensions[2]+half_dist
    New_y2=dimensions[3]+half_dist

    if New_x1<0:
        New_x2+=abs(New_x1)
        New_x1=0

    if New_x2>640:
        New_x1-=abs(New_x2-640)
        New_x2=640
    if New_y1<0:
        New_y2+=abs(New_y1)
        New_y1=0

    if New_y2>480:
        New_y1-=abs(New_y2-480)
        New_y2=480

    max_edge = max(xy_dims)
    to_correct = max_edge*crop_scale

    if xy_idx == 0:
        return [dimensions[0], New_y1, dimensions[2], New_y2]
    else:
        return [New_x1, dimensions[1], New_x2, dimensions[3]]


def crop_img(img,crop_XYXY):

    return img[crop_XYXY[1]:crop_XYXY[3],crop_XYXY[0]:crop_XYXY[2],:]


def resize_img(img,size):

    raise NotImplementedError

def draw_outputs(myimage, outputs):
    for pred_class, bounding_box, pred_angle, pred_kps_center in outputs:
        center_coordinates = (int(pred_kps_center[0]),int(pred_kps_center[1]))
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

def image_analyze_stream(msg_id):

    global object_locator
    global angle_predictor
    global rgb_image
    global detection_pub
    global bounding_box
    global pred_angle
    global pred_kps_center

    t = threading.currentThread()
    dt2_correction_flag=0

    analyze_img=rgb_image.copy()
    (winW, winH) = (224, 224)
    timer=0
    
    result = []
    msg = Detections()
    for instance, pred_class, bounding_box, pred_angle, pred_kps_center in object_locator.predict(analyze_img):
        detection = Detection()
        bounding_box=np.asarray(bounding_box)
        bounding_box=bounding_box.astype(int)
        
        if (instance > 0):
            x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
            ctr_X = int((bounding_box[0]+bounding_box[2])/2)
            ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
            angle = pred_angle
            ref_x = 640/2
            ref_y = 480/2
            dist = [ctr_X - ref_x, ref_y - ctr_Y]
            dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
            # x_robot, y_robot = convert_detection_pose(dist[0], dist[1])
            # msg.x = x_robot 
            # msg.y = y_robot
            detection.bounding_box = x1, y1, x2, y2
            detection.kps_x = pred_kps_center[0]
            detection.kps_y = pred_kps_center[1]
            detection.angle = pred_angle
            detection.obj_class = pred_class

            msg.detections.append(detection)
            result.append([pred_class, bounding_box, pred_angle, pred_kps_center])
    if result:
        detection_pub.publish(msg)
        return result
    else:
        return [[9999,9999,9999,9999],9999,[9999,9999]]
    '''
    elif (msg_id == 2):
        instance, bounding_box, pred_angle, pred_kps_center = object_locator2.predict(analyze_img)
        bounding_box=np.asarray(bounding_box)
        bounding_box=bounding_box.astype(int)
        msg = Float32MultiArray()
        if (instance > 0):

            x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
            ctr_X = int((bounding_box[0]+bounding_box[2])/2)
            ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
            angle = pred_angle
            ref_x = 640/2
            ref_y = 480/2
            dist = [ctr_X - ref_x, ref_y - ctr_Y]
            dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]


            return [bounding_box, pred_angle, pred_kps_center]
        else:
            return [[9999,9999,9999,9999],9999,[9999,9999]]
	'''

def convert_detection_pose(x ,y):
    listener_tf = tf.TransformListener()
    camera_focal = 550
    (trans1, rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    z_to_surface = trans1[2]
    to_world_scale = z_to_surface / camera_focal

    x_dist = x * to_world_scale
    y_dist = y * to_world_scale

    my_point = PoseStamped()
    my_point.header.frame_id = "camera_color_frame"
    my_point.header.stamp = rospy.Time(0)
    my_point.pose.position.x = 0
    my_point.pose.position.y = -x_dist
    my_point.pose.position.z = y_dist
    theta = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    my_point.pose.orientation.x = quat[0]
    my_point.pose.orientation.y = quat[1]
    my_point.pose.orientation.z = quat[2]
    my_point.pose.orientation.w = quat[3]
    ps = listener_tf.transformPose("/panda_link0", my_point)

    (trans, rot) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    data = (ps.pose.position.x - trans[0], ps.pose.position.y - trans[1])

    return [data[0], data[1]]


def image_analyze(msg):

    global object_locator
    # global angle_predictor
    global rgb_image
    global pub
    global bounding_box
    global pred_angle
    global pred_kps_center

    t = threading.currentThread()
    dt2_correction_flag=0

    analyze_img=rgb_image.copy()
    (winW, winH) = (224, 224)
    timer=0

    for i in range(10):
        instance, pred_class, bounding_box, pred_angle, pred_kps_center = object_locator.predict(analyze_img)
        if (instance > 0):
            break

    bounding_box=np.asarray(bounding_box)
    bounding_box=bounding_box.astype(int)
    response = GraspPoseDetectionResponse()
    if (instance > 0):
        x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
        ctr_X = int((bounding_box[0]+bounding_box[2])/2)
        ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
        angle = pred_angle
        ref_x = 640/2
        ref_y = 480/2
        dist = [ctr_X - ref_x, ref_y - ctr_Y]
        dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
        response.detection.obj_class = pred_class
        response.detection.x = dist[0]
        response.detection.y = dist[1]
        response.detection.angle = angle
        response.detection.kps_x = dist_kps_ctr[0]
        response.detection.kps_y = dist_kps_ctr[1]
    else:
        response.detection.x = 9999
        response.detection.y = 9999
        response.detection.angle = 9999
        response.detection.kps_x = 9999
        response.detection.kps_y = 9999
    return response


    '''
    elif (msg_id == 2):
        instance, bounding_box, pred_angle, pred_kps_center = object_locator2.predict(analyze_img)
        bounding_box=np.asarray(bounding_box)
        bounding_box=bounding_box.astype(int)
        msg = Float32MultiArray()
        if (instance > 0):

            x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
            ctr_X = int((bounding_box[0]+bounding_box[2])/2)
            ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
            angle = pred_angle
            ref_x = 640/2
            ref_y = 480/2
            dist = [ctr_X - ref_x, ref_y - ctr_Y]
            dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
            msg.data = [msg_id, dist[0], dist[1], angle, dist_kps_ctr[0], dist_kps_ctr[1]]
            print (msg.data)
            pub.publish(msg)
	'''




if __name__ == '__main__':

    rospy.init_node('grasp_server', anonymous=True)
    object_locator = detectron_model("/home/opendr/catkin_ws/src/cobot_vision/models/metrics_model.pth")
    object_num = 1
    rgb_image=cv2.imread("samples/image.png")
    
    detection_server = rospy.Service('/detect_grasp_pose', GraspPoseDetection, image_analyze)

    detection_pub = rospy.Publisher('/objects_detected', Detections, queue_size=10)

    # detection_request_sub = rospy.Subscriber("/request_detection", Int16, request_callback)
    sub=  rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()
