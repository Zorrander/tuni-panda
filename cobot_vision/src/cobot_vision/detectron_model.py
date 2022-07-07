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

class DetectronModel(object):

    def __init__(self, model_filepath):
        self.model_filepath = model_filepath
        self.load_graph(model_filepath = self.model_filepath)

    def load_graph(self, model_filepath):
        self.cfg=get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.2
        self.cfg.SOLVER.BASE_LR = 0.0008
        self.cfg.SOLVER.MAX_ITER = 1000
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
        self.cfg.MODEL.ROI_KEYPOINT_HEAD.NUM_KEYPOINTS = 12
        self.cfg.MODEL.DEVICE="cpu"
        self.cfg.INPUT.MIN_SIZE_TEST=0
        self.cfg.MODEL.WEIGHTS = os.path.join("",self.model_filepath)
        self.predictor=DefaultPredictor(self.cfg)
        print('Model loading complete!')



    def correct_orientation_ref(self, angle):

        if angle <= 90:
            angle += 90
        if angle > 90:
            angle += -270
        return angle

    def get_angle(self, input_kps, mode):

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


    def get_kps_center(self, input_kps) :
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
            result.append([1, pred_classes[i], np.array(bounding_box[i]), self.correct_orientation_ref(self.get_angle(keypoints_pred[i],1)), self.get_kps_center(keypoints_pred[i])])

        else:
            print("No detection")
            result.append([0, 1e10, [-1,-1,-1,-1] , -1, [-1, -1]])
        return result