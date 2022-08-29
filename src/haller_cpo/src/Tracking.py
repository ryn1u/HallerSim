import pyzed.sl as sl
import numpy as np
import sys
import cv2 as cv

from mask_tuning import mask_bounds
from img_preprocess import *
#from countur_detect import *
import torch
from torchvision.ops import nms
import datetime
import pandas as pd
import time
import typing


class TrackAndInf:


    xcen_of_view = 540
    ycen_of_view = 360

    @staticmethod
    def track_object(b_box, depth_frame):
        # global delay
        # global x_deviation, y_deviation, tolerance, arr_track_data

        # x_diff = x_max - x_min
        # y_diff = y_max - y_min
        # print("x_diff: ", round(x_diff, 5))
        # print("y_diff: ", round(y_diff, 5))

        obj_x_center = b_box[0] + (b_box[2] / 2)
        obj_x_center = round(obj_x_center, 3)

        obj_y_center = b_box[1] + (b_box[3] / 2)
        obj_y_center = round(obj_y_center, 3)

        # print("[",obj_x_center, obj_y_center,"]")

        x_deviation = round(TrackAndInf.xcen_of_view - obj_x_center, 3)
        y_deviation = round(TrackAndInf.ycen_of_view - obj_y_center, 3)

        dist = round(depth_frame[int(obj_y_center), int(obj_x_center)], 3)
        # print("{", x_deviation, y_deviation, "}")

        # move_robot()
        # thread = Thread(target=move_robot)
        # thread.start()
        # thread.join()

        # print(cmd)

        arr_track_data = [obj_x_center, obj_y_center, x_deviation, y_deviation, dist]
        # arr_track_data[1] = obj_y_center
        # arr_track_data[2] = x_deviation
        # arr_track_data[3] = y_deviation
        return arr_track_data

    @staticmethod
    def draw_overlays(cv2_im, b_box, arr_dur, arr_track_data, label, prob):
        height, width, channels = cv2_im.shape
        font = cv.FONT_HERSHEY_SIMPLEX

        # global tolerance
        tolerance = 0.1

        # draw black rectangle on top
        cv2_im = cv.rectangle(cv2_im, (0, 0), (width, 24), (0, 0, 0), -1)

        #FPS nie dziala w symulacji, wiec nie wstawiam tych info
        # write processing durations
        cam = round(arr_dur[0] * 1000, 0)
        inference = round(arr_dur[1] * 1000, 0)
        other = round(arr_dur[2] * 1000, 0)
        text_dur = 'Camera: {}ms   Inference: {}ms   other: {}ms'.format(cam, inference, other)
        #cv2_im = cv.putText(cv2_im, text_dur, (int(width / 4) - 30, 16), font, 0.4, (255, 255, 255), 1)

        # write FPS
        total_duration = cam + inference + other
        fps = round(1000 / total_duration, 1)
        text1 = 'FPS: {}'.format(fps)
        #cv2_im = cv.putText(cv2_im, text1, (10, 20), font, 0.7, (150, 150, 255), 2)

        if label == "Tracking":
            label_color = (0, 0, 255)
        else:
            label_color = (0, 255, 0)

        # Write found object
        text_state = 'Object: {}'.format(label)
        cv2_im = cv.putText(cv2_im, text_state, (10, 20), font, 0.55, label_color, 2)

        # Write probability
        text_prob = 'Probability: {}'.format(prob)
        cv2_im = cv.putText(cv2_im, text_prob, (300, 20), font, 0.55, label_color, 2)


        # draw black rectangle at bottom
        cv2_im = cv.rectangle(cv2_im, (0, height - 24), (width, height), (0, 0, 0), -1)

        # write deviations and tolerance
        str_tol = 'Tol X: {} Tol Y: {}'.format(tolerance * width, tolerance * height)
        cv2_im = cv.putText(cv2_im, str_tol, (10, height - 8), font, 0.55, (150, 150, 255), 2)

        x_dev = arr_track_data[2]
        str_x = 'X: {}'.format(x_dev)
        if (abs(x_dev) < tolerance * width):
            color_x = (0, 255, 0)
        else:
            color_x = (0, 0, 255)
        cv2_im = cv.putText(cv2_im, str_x, (250, height - 8), font, 0.55, color_x, 2)

        y_dev = arr_track_data[3]
        str_y = 'Y: {}'.format(y_dev)
        if (abs(y_dev) < tolerance * height):
            color_y = (0, 255, 0)
        else:
            color_y = (0, 0, 255)
        cv2_im = cv.putText(cv2_im, str_y, (360, height - 8), font, 0.55, color_y, 2)

        str_dist = 'Dist: {}'.format(arr_track_data[4])
        cv2_im = cv.putText(cv2_im, str_dist, (460, height - 8), font, 0.55, (150, 150, 255), 2)

        # write direction, speed, tracking status
        # cmd = arr_track_data[4]
        # cv2_im = cv.putText(cv2_im, str(cmd), (int(width / 2) + 10, height - 8), font, 0.68, (0, 255, 255), 2)

        # delay1 = arr_track_data[5]
        # str_sp = 'Speed: {}%'.format(round(delay1 / (0.1) * 100, 1))
        # cv2_im = cv.putText(cv2_im, str_sp, (int(width / 2) + 185, height - 8), font, 0.55, (150, 150, 255), 2)

        # if (cmd == 0):
        #   str1 = "No object"
        # elif (cmd == 'Stop'):
        #    str1 = 'Acquired'
        # else:
        #    str1 = 'Tracking'
        # cv2_im = cv2.putText(cv2_im, str1, (width - 140, 18), font, 0.7, (0, 255, 255), 2)

        # draw center cross lines
        cv2_im = cv.rectangle(cv2_im, (0, int(height / 2) - 1), (width, int(height / 2) + 1), (255, 0, 0), -1)
        cv2_im = cv.rectangle(cv2_im, (int(width / 2) - 1, 0), (int(width / 2) + 1, height), (255, 0, 0), -1)

        # draw the center red dot on the object
        cv2_im = cv.circle(cv2_im, (int(arr_track_data[0]), int(arr_track_data[1])), 7, (0, 0, 255), -1)

        # draw the tolerance box
        cv2_im = cv.rectangle(cv2_im, (int(width / 2 - tolerance * width), int(height / 2 - tolerance * height)),
                              (int(width / 2 + tolerance * width), int(height / 2 + tolerance * height)), (0, 0, 255),
                              2)
        """
        # draw bounding boxes
        for obj in objs:
            x0, y0, x1, y1 = list(obj.bbox)
            x0, y0, x1, y1 = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)
            percent = int(100 * obj.score)

            box_color, text_color, thickness = (0, 150, 255), (0, 255, 0), 2
            cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), box_color, thickness)

            # text3 = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
            # cv2_im = cv2.putText(cv2_im, text3, (x0, y1-5),font, 0.5, text_color, thickness)
        """
        box_color, text_color, thickness = (0, 150, 255), (0, 255, 0), int(2)
        cv2_im = cv.rectangle(cv2_im, (b_box[0], b_box[1]), (b_box[0] + b_box[2], b_box[1] + b_box[3]), text_color,
                              thickness)

        return cv2_im