import pyzed.sl as sl
import numpy as np
import sys
import cv2 as cv

from mask_tuning import mask_bounds
from img_preprocess import masked_img
#from countur_detect import *
import torch
from torchvision.ops import nms
import datetime
import pandas as pd
import time
import typing


class ZedCam:

    def __init__(self):
        self.zed = sl.Camera()
        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
        self.init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        self.init_params.camera_resolution = sl.RESOLUTION.HD720

        # Open the camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # Use STANDARD sensing mode
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        # Create an RGBA sl.Mat object
        self.image_zed = sl.Mat(self.zed.get_camera_information().camera_resolution.width,
                           self.zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
        # Retrieve data in a numpy array with get_data()
        self.image_ocv = self.image_zed.get_data()

        # Create a sl.Mat with float type (32-bit)
        self.depth_zed = sl.Mat(self.zed.get_camera_information().camera_resolution.width,
                           self.zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)

        # Create an RGBA sl.Mat object
        self.image_depth_zed = sl.Mat(self.zed.get_camera_information().camera_resolution.width,
                                 self.zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

        self.image_depth_ocv = None
        self.depth_ocv = None

    def zed_view(self):
        #       NORMAL IMAGE
        # Retrieve the left image in sl.Mat
        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        # Use get_data() to get the numpy array
        self.image_ocv = self.image_zed.get_data()
        # Display the left image from the numpy array
        #cv.imshow("Image", self.image_ocv)

        #       DEPTH SENSING - FOR FURTHER PROCESSING
        # Retrieve depth data (32-bit)
        self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)
        # Load depth data into a numpy array
        self.depth_ocv = self.depth_zed.get_data()
        # Print the depth value at the center of the image
        #print(depth_ocv[int(len(depth_ocv) / 2)][int(len(depth_ocv[0]) / 2)])

        #       DEPTH SENSING IMAGE - ONLY FOR DISPLAY
        # Retrieve the normalized depth image
        self.zed.retrieve_image(self.image_depth_zed, sl.VIEW.DEPTH)
        # Use get_data() to get the numpy array
        self.image_depth_ocv = self.image_depth_zed.get_data()
        # Display the depth view from the numpy array
        #cv.imshow("Image", np.hstack([image_depth_ocv,image_ocv]))

"""
zed_cam = ZedCam()

while True:    #True:
    start_time = time.time()
    start_t0 = time.time()
    #IMAGE AND DEPTH
    if zed_cam.zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed_cam.zed_view()

    if True: #do_u_want_zed == 1:
        frame = zed_cam.image_ocv[:, :, 0:3]
        image_depth = zed_cam.image_depth_ocv[:, :, 0:3]
        depth_frame = zed_cam.depth_ocv

    cv.imshow("Image", frame)
    if cv.waitKey(1) == ord('q'):
        break
        """