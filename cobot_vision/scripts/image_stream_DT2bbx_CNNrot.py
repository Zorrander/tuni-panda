#!/usr/bin/env python
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

CUDA_LAUNCH_BLOCKING=1

def callback(image_data):
    global rgb_image
    # rgb_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="rgb8")
    rgb_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width,3)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    cv2.imshow("test",rgb_image)
    cv2.waitKey(1)




def request_callback(data):

    print ("new request:")
    print (data.data)
    image_analyze(data.data)


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


def image_analyze(msg_id):

    global object_locator1,  object_locator2
    global angle_predictor
    global rgb_image
    global pub
    t = threading.currentThread()
    dt2_correction_flag=0

    analyze_img=rgb_image.copy()
    (winW, winH) = (224, 224)
    timer=0

    if (msg_id == 1):
        instance, bounding_box, pred_angle, pred_kps_center = object_locator1.predict(analyze_img)
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

            #analyze_img = cv2.rectangle(analyze_img, (x1, y1), (x2, y2), (255,0,0), 2)
            # cv2.putText(analyze_img, str(angle), (550,470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            # cv2.imshow("Window", analyze_img)
            # cv2.imshow("crop", square_crop_img)
            # cv2.waitKey(1)
        else:
            msg.data=[msg_id,9999,9999,9999,9999]
            pub.publish(msg)


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

            #analyze_img = cv2.rectangle(analyze_img, (x1, y1), (x2, y2), (255,0,0), 2)
            # cv2.putText(analyze_img, str(angle), (550,470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            # cv2.imshow("Window", analyze_img)
            # cv2.imshow("crop", square_crop_img)
            # cv2.waitKey(1)
        else:
            msg.data=[msg_id,9999,9999,9999,9999]
            pub.publish(msg)

    # elif (msg_id == 3):
    #     instance, bounding_box, pred_angle, pred_kps_center = object_locator3.predict(analyze_img)
    #     bounding_box=np.asarray(bounding_box)
    #     bounding_box=bounding_box.astype(int)
    #     msg = Float32MultiArray()
    #     if (instance > 0):
    #
    #         x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
    #         ctr_X = int((bounding_box[0]+bounding_box[2])/2)
    #         ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
    #         angle = pred_angle
    #         ref_x = 640/2
    #         ref_y = 480/2
    #         dist = [ctr_X - ref_x, ref_y - ctr_Y]
    #         dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
    #         msg.data = [msg_id, dist[0], dist[1], angle, dist_kps_ctr[0], dist_kps_ctr[1]]
    #         print (msg.data)
    #         pub.publish(msg)
    #
    #         #analyze_img = cv2.rectangle(analyze_img, (x1, y1), (x2, y2), (255,0,0), 2)
    #         # cv2.putText(analyze_img, str(angle), (550,470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    #         # cv2.imshow("Window", analyze_img)
    #         # cv2.imshow("crop", square_crop_img)
    #         # cv2.waitKey(1)
    #     else:
    #         msg.data=[msg_id,9999,9999,9999,9999]
    #         pub.publish(msg)



if __name__ == '__main__':

    rospy.init_node('grasp_server',
                        anonymous=True)
    object_locator1 = detectron_model("models/model_final.pth")
    object_locator2 = detectron_model("models/piston.pth")
    object_locator3 = detectron_model("models/commonline.pth")

    rgb_image=cv2.imread("samples/image.png")
    pub= rospy.Publisher('/commands', Float32MultiArray, queue_size=10)
    detection_request_sub = rospy.Subscriber("/request_detection", Int16, request_callback)
    sub=  rospy.Subscriber("/camera/color/image_raw", Image, callback)
    input()
