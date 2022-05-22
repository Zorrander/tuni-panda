#!/usr/bin/env python3

import sys
import detectron2
from detectron2.utils.logger import setup_logger
import cv2
import matplotlib.pyplot as plt
import time
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
from detectron2 import model_zoo
import os
import numpy as np
import json
from detectron2.structures import BoxMode
import itertools
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode
import random
import time
import math
import pandas as pd
from collections import Counter

def rot(phi):

  phi = np.deg2rad(phi)
  return np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])

def drawRect(img, rectangle):

    x, y, w, h, angle = rectangle[0], rectangle[1], rectangle[2], rectangle[3], -rectangle[4]
    cloned_img=img.copy()
    a = np.array((-w / 2, -h / 2))
    b = np.array((w / 2, -h / 2))
    c = np.array((w / 2, h / 2))
    d = np.array((-w / 2, h / 2))

    if angle != 0:
        a = np.matmul(rot(angle), a)
        b = np.matmul(rot(angle), b)
        c = np.matmul(rot(angle), c)
        d = np.matmul(rot(angle), d)

    a += [x, y]
    b += [x, y]
    c += [x, y]
    d += [x, y]
    a=a.astype(int)
    b=b.astype(int)
    c=c.astype(int)
    d=d.astype(int)

    cv2.line(cloned_img, (a[0],a[1]), (b[0],b[1]), (0,255,0), 2)
    cv2.line(cloned_img, (b[0],b[1]), (c[0],c[1]), (0,255,0), 2)
    cv2.line(cloned_img, (c[0],c[1]), (d[0],d[1]), (0,255,0), 2)
    cv2.line(cloned_img, (d[0],d[1]), (a[0],a[1]), (0,255,0), 2)

    return cloned_img

def get_straight_bbox(img, rectangle):
    x, y, w, h, angle = rectangle[0], rectangle[1], rectangle[2], rectangle[3], -rectangle[4]
    cloned_img=img.copy()
    long_edge=int(max((w,h))/2*1.2)
    cropped_img = cloned_img[(y-long_edge):(y+long_edge), (x-long_edge):(x+long_edge),:]
    return cropped_img


def correct_orientation_ref(angle):

    if angle <= 90:
        angle += 90
    if angle > 90:
        angle += -270

    return angle

def get_angle(input_kps, mode):

    # kps_x_list = input_kps[0][:,0]
    # kps_y_list = input_kps[0][:,1]
    kps_x_list = input_kps[:,0]
    kps_y_list = input_kps[:,1]
    kps_x_list = kps_x_list[::-1]
    kps_y_list = kps_y_list[::-1]

    d = {'X' : kps_x_list,
        'Y' : kps_y_list}

    df = pd.DataFrame(data = d)
    # move the origin
    x = (df["X"] - df["X"][0])
    y = (df["Y"] - df["Y"][0])

    if mode == 1:

        list_xy = (np.arctan2(y,x)*180/math.pi).astype(int)
        occurence_count = Counter(list_xy)
        return occurence_count.most_common(1)[0][0]
    else:
        x = np.mean(x)
        y= np.mean(y)
        return np.arctan2(y,x)*180/math.pi

def get_kps_center(input_kps) :

    # kps_x_list = input_kps[0][:,0]
    # kps_y_list = input_kps[0][:,1]
    kps_x_list = input_kps[:,0]
    kps_y_list = input_kps[:,1]
    kps_x_list = kps_x_list[::-1]
    kps_y_list = kps_y_list[::-1]
    d = {'X' : kps_x_list,
        'Y' : kps_y_list}
    df = pd.DataFrame(data = d)

    x = np.mean(df["X"])
    y= np.mean(df["Y"])
    return [x,y]


class detectron_model(object):

    def __init__(self, model_filepath):
        self.model_filepath = model_filepath
        self.load_graph(model_filepath = self.model_filepath)

    def load_graph(self, model_filepath):
        self.cfg=get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.3
        self.cfg.SOLVER.BASE_LR = 0.0008
        self.cfg.SOLVER.MAX_ITER = 1000
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
        self.cfg.MODEL.ROI_KEYPOINT_HEAD.NUM_KEYPOINTS = 12
        self.cfg.MODEL.DEVICE="cuda"
        self.cfg.INPUT.MIN_SIZE_TEST=0
        self.cfg.MODEL.WEIGHTS = os.path.join("",self.model_filepath)
        self.predictor=DefaultPredictor(self.cfg)
        print('Model loading complete!')

    def predict(self, data):
        result = [] 

        output=self.predictor(data)
        #print(len(output["instances"]))
        # print(output["instances"].to("cpu").pred_classes.numpy())
        pred_classes = output["instances"].to("cpu").pred_classes.numpy()
        bounding_box = output["instances"].to("cpu").pred_boxes.tensor.numpy()
        keypoints_pred =  output["instances"].to("cpu").pred_keypoints.numpy()

        for i in range(len(bounding_box)):
            #if pred_classes[i]==0:
            result.append([1, pred_classes[i], np.array(bounding_box[i]), correct_orientation_ref(get_angle(keypoints_pred[i],1)), get_kps_center(keypoints_pred[i])])

        else:
            print("No detection")
            result.append([0, 1e10, [-1,-1,-1,-1] , -1, [-1, -1]])
        return result

if __name__ == '__main__':
    #
    # instance, bounding_box, pred_angle, pred_kps_center = object_locator.predict(analyze_img)
    # bounding_box=np.asarray(bounding_box)
    # bounding_box=bounding_box.astype(int)

    # msg = Float32MultiArray()

    # if (instance > 0):
        # print (bounding_box)
        # x1, y1, x2, y2 = bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
        # ctr_X = int((bounding_box[0]+bounding_box[2])/2)
        # ctr_Y = int((bounding_box[1]+bounding_box[3])/2)
        # angle = pred_angle
        # ref_x = 640/2
        # ref_y = 480/2
        # dist = [ctr_X - ref_x, ref_y - ctr_Y]
        # dist_kps_ctr = [pred_kps_center[0] - ref_x, ref_y - pred_kps_center[1]]
        # msg.data = [msg_id, dist[0], dist[1], angle, dist_kps_ctr[0], dist_kps_ctr[1]]
        # pub.publish(msg)
    myimage = cv2.imread("samples/image.png")
    clone = myimage.copy()
    model = detectron_model("models/metrics_model.pth")
    time1 = time.time()
    instance, bounding_box, pred_angle, pred_kps_center = model.predict(myimage)
    #print(time.time()-time1)

    center_coordinates = (int(pred_kps_center[0])-320, 240-int(pred_kps_center[1]))
    print (center_coordinates)
    radius = 10
    color = (255, 0, 0)
    thickness = 2
    clone = cv2.circle(myimage, center_coordinates, radius, color, thickness)

    out_img = cv2.rectangle(clone, (int(bounding_box[0]), int(bounding_box[1])), (int(bounding_box[2]), int(bounding_box[3])), (255,0,0), 2)
    cv2.imshow("lalala", out_img)
    cv2.waitKey(5000)
