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


class ZedPosition:

    def __init__(self):
        self.init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                       coordinate_units=sl.UNIT.METER,
                                       coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD)

        # If applicable, use the SVO given as parameter
        # Otherwise use ZED live stream
        if len(sys.argv) == 2:
            filepath = sys.argv[1]
            print("Using SVO file: {0}".format(filepath))
            self.init_params.set_from_svo_file(filepath)

        self.zed = sl.Camera()
        self.status = self.zed.open(self.init_params)
        if self.status != sl.ERROR_CODE.SUCCESS:
            print(repr(self.status))
            exit()

        self.tracking_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(self.tracking_params)

        self.runtime = sl.RuntimeParameters()
        self.camera_pose = sl.Pose()

        self.camera_info = self.zed.get_camera_information()
        # Create OpenGL viewer
        self.viewer = gl.GLViewer()
        self.viewer.init(self.camera_info.camera_model)

        self.py_translation = sl.Translation()
        self.pose_data = sl.Transform()

        self.text_translation = ""
        self.text_rotation = ""
        self.zed_pose = sl.Pose()
        self.tx = None
        self.ty = None
        self.tz = None
        self.ox = None
        self.oy = None
        self.oz = None
        self.ow = None

    def zed_position(self):
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = self.zed.get_position(self.camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = self.camera_pose.get_rotation_vector()
                translation = self.camera_pose.get_translation(self.py_translation)
                self.text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                self.text_translation = str(
                    (round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                self.pose_data = self.camera_pose.pose_data(sl.Transform())
            self.viewer.updateData(self.pose_data, self.text_translation, self.text_rotation, tracking_state)
            """Additional position tracking"""
            # Get the pose of the camera relative to the world frame
            self.state = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
            # Display translation and timestamp
            py_translation = sl.Translation()
            self.tx = round(self.zed_pose.get_translation(py_translation).get()[0], 2)
            self.ty = round(self.zed_pose.get_translation(py_translation).get()[1], 2)
            self.tz = round(self.zed_pose.get_translation(py_translation).get()[2], 2)
            # print("Translation: tx: {0}, ty:  {1}, tz:  {2}, timestamp: {3}\n".format(tx, ty, tz, self.zed_pose.timestamp))
            # Display orientation quaternion
            py_orientation = sl.Orientation()
            self.ox = round(self.zed_pose.get_orientation(py_orientation).get()[0], 2)
            self.oy = round(self.zed_pose.get_orientation(py_orientation).get()[1], 2)
            self.oz = round(self.zed_pose.get_orientation(py_orientation).get()[2], 2)
            self.ow = round(self.zed_pose.get_orientation(py_orientation).get()[3], 2)
            # print("Orientation: ox: {0}, oy:  {1}, oz: {2}, ow: {3}\n".format(ox, oy, oz, ow))


zed_pos = ZedPosition()

while zed_pos.viewer.is_available():
    zed_pos.zed_position()
    print(zed_pos.tx)