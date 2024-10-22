#!/usr/bin/env python3

import sys
import cv2
import os
import numpy as np
import torch

class YoloDetector:
    def __init__(self):
        # Model
        cache_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"ultralytics_yolov5_master")
        self.model = torch.hub.load(cache_path, 'custom',path=cache_path+"/yolov5m6",source='local')
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s',pretrained=True, classes=80)
        self.model.conf = 0.7  # confidence threshold (0-1)
        self.model.iou = 0.7  # NMS IoU threshold (0-1)
        self.model.classes = [0] #[0]  # (optional list) filter by class, i.e. = [0, 15, 16] for persons, cats and dogs
        self.results = ''

    def detect_person(self, rgb_img):
        self.results = self.model(rgb_img)
        self.results.render()
        return self.results.xyxy[0].tolist()  # [[xmin, ymin, xmax, ymax, score, class]]

    """
    # without filter
    def detect_person(self, rgb_img):
        self.results = self.model(rgb_img)
        self.results.render()
        persons = [p for p in self.results.xyxy[0].tolist() if p[5] == 0.0]
        return persons  # [[xmin, ymin, xmax, ymax, score, class]]
    """

    def get_rendered(self):
        return self.results.imgs[0]  #rgb


# Utilized function
def cut_box(img, box):
    xmin, ymin, xmax, ymax, _, _ = box
    out = img[int(ymin):int(ymax), int(xmin):int(xmax)]
    return out

def cut_boxes(img, boxes):
    out_list = []
    for box in boxes:
        xmin, ymin, xmax, ymax, _, _ = box
        out = img[int(ymin):int(ymax), int(xmin):int(xmax)]
        out_list.append(out)
    return out_list

def draw_box(img, box, color=(0,0,255), width=3):
    xmin, ymin, xmax, ymax, _, _ = box
    start_point = (int(xmin), int(ymin))
    end_point = (int(xmax), int(ymax))
    cv2.rectangle(img, start_point, end_point, color, width)
    return img

def draw_boxes(img, boxes, color=(0,0,255), width=3):
    for box in boxes:
        xmin, ymin, xmax, ymax, _, _ = box
        start_point = (int(xmin), int(ymin))
        end_point = (int(xmax), int(ymax))
        cv2.rectangle(img, start_point, end_point, color, width)
    return img

