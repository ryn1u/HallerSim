#!/usr/bin/env python
import rospy
from typing import *

import utils
from utils.base_ros_handler import BaseROSHandler

from geometry_msgs.msg import Twist, Vector3
from detection_msgs.msg import BoundingBoxes

import cv2
import numpy as np


import pyzed.sl as sl
import sys
from mask_tuning import mask_bounds
from img_preprocess import *
# from countur_detect import *
import torch
from torchvision.ops import nms
import datetime
import pandas as pd
import time
import typing
from ZED_video import *
from Tracking import *

"""
SIMULATION STARTING SEQUENCE:
1. Make sure simulationConfig.json files in Unity and ROS are the same and are latest version.
2. Start ROS server with:
    cd *insert path*/HallerSim
    source devel/setup.bash
    roslaunch haller_cpo simulation.launch
3. Start Unity
4. Check if IP is correct and click 'Connect to ROS'
5. Click Start Simulation (Screen might turn black and white! It's OK!
6. Run this file (preferably use pyCharm)

7. (optional) Check if data is flowing through topics:
    - for text:
        'rostopic list'
        'rostopic echo *topic name*'
    - for images:
        'rosrun rqt_image_viewer rqt_image_viewer'
"""


class HallerCPO(BaseROSHandler):
    """
    Class for handling CPO. BaseROSHandler does basic ROS system initialization and basic depth data processing.
    See BaseROSHandler code in utils/base_ros_handler.py
    """
    def __init__(self):
        super().__init__()
        # Here goes all initialization code you need to implement.
        pass

    def update(self, current_position, video_image, depth_data, bounding_boxes: BoundingBoxes):
        # here goes CPO code. this method is called every second
        # angles are in degrees
        x, y, z, roll, pitch, yaw = current_position
        # fps = 1
        do_u_want_zed = 1
        perform_mask_tuning = 0

        arr_dur = [1, 1, 1]

        if perform_mask_tuning:
            lower_bound, upper_bound = mask_bounds()
        else:
            lower_bound = np.array([0, 130, 117])
            upper_bound = np.array([255, 139, 152])

        start_time = time.time()
        start_t0 = time.time()

        frame = haller_cpo_handler.video_image

        image_depth = haller_cpo_handler.depth_data
        depth_frame = haller_cpo_handler.depth_data

        arr_dur[0] = time.time() - start_t0

        start_t1 = time.time()

        masked_frame, masked_frame_in_BGR, enhanced_LAB_into_BGR = masked_img(frame, lower_bound, upper_bound)
        #b_box = contour_detection(masked_frame, enhanced_LAB_into_BGR)

        if bounding_boxes:
            for obj in bounding_boxes:
                if obj.Class == "bootleger":
                    b_box = [obj.xmin, obj.ymin, obj.xmax - obj.xmin, obj.ymax - obj.ymin]
                    label = obj.Class
                    prob = obj.probability
        else:
            b_box = [540, 360, 0, 0]
            label = "Tracking"
            prob = 0


        arr_track_data = TrackAndInf.track_object(b_box, depth_frame)
        # frame = np.ascontiguousarray(frame, dtype=np.uint8)
        cv2_im = TrackAndInf.draw_overlays(frame, b_box, arr_dur, arr_track_data, label, prob)
        cv2.imshow("image", cv2_im)
        arr_dur[1] = time.time() - start_t1

        start_t2 = time.time()

        arr_dur[2] = time.time() - start_t2
        # cv.namedWindow('Image2', cv.WINDOW_NORMAL)
        # cv.resizeWindow('Image2', 1000, 500)
        # fps = round(1.0 / (time.time() - start_time), 1)
        # cv.imshow("orange_detection", np.hstack([enhanced_LAB_into_BGR, masked_frame_in_BGR]))
        arr_dur = [1, 1, 1]
        if cv.waitKey(1) == ord('q'):
            print("q")

    def send_target_position(self, x, y, z, roll, pitch, yaw):
        # Sends new target position to simulation. Remember to use North-East-Down coordinates
        msg = Twist(Vector3(x, y, z), Vector3(roll, pitch, yaw))
        self.publishers['simTargetPosition'].publish(msg)

    def send_delta_position(self, x, y, z, roll, pitch, yaw):
        # Sends new target position to simulation. Remember to use North-East-Down coordinates
        msg = Twist(Vector3(x, y, z), Vector3(roll, pitch, yaw))
        self.publishers['simDeltaPosition'].publish(msg)


if __name__ == '__main__':
    haller_cpo_handler = HallerCPO()
    haller_cpo_handler.start()



