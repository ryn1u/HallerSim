import pyzed.sl as sl
import numpy as np
import sys
import cv2 as cv

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


# fps = 1
do_u_want_zed = 1
perform_mask_tuning = 0

arr_dur = [0, 0, 0]

if perform_mask_tuning:
    lower_bound, upper_bound = mask_bounds()
else:
    lower_bound = np.array([0, 130, 81])
    upper_bound = np.array([240, 255, 126])

zed_cam = ZedCam()



while True:
    start_time = time.time()
    start_t0 = time.time()

    if zed_cam.zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed_cam.zed_view()

    if do_u_want_zed == 1:
        frame = zed_cam.image_ocv[:, :, 0:3]
        image_depth = zed_cam.image_depth_ocv[:, :, 0:3]
        depth_frame = zed_cam.depth_ocv

    arr_dur[0] = time.time() - start_t0

    start_t1 = time.time()

    masked_frame, masked_frame_in_BGR, enhanced_LAB_into_BGR = masked_img(frame, lower_bound,upper_bound)
    b_box = contour_detection(masked_frame, enhanced_LAB_into_BGR)
    arr_track_data = TrackAndInf.track_object(b_box, depth_frame)
    frame = np.ascontiguousarray(frame, dtype=np.uint8)
    cv2_im = TrackAndInf.draw_overlays(frame, b_box, arr_dur, arr_track_data)
    cv.imshow("image", cv2_im)
    cv.imshow("image2", image_depth)
    arr_dur[1] = time.time() - start_t1

    start_t2 = time.time()

    arr_dur[2] = time.time() - start_t2
    # cv.namedWindow('Image2', cv.WINDOW_NORMAL)
    # cv.resizeWindow('Image2', 1000, 500)
    # fps = round(1.0 / (time.time() - start_time), 1)
    # cv.imshow("orange_detection", np.hstack([enhanced_LAB_into_BGR, masked_frame_in_BGR]))

    if cv.waitKey(1) == ord('q'):
        break

zed_cam.zed.close()
