import pyzed.sl as sl
import numpy as np
import sys
import cv2 as cv

from mask_tuning import mask_bounds
from img_preprocess import *
#from countur_detect import contour_detection
import torch
from torchvision.ops import nms
import datetime
import pandas as pd
import time
import typing

do_u_want_zed = 1
perform_mask_tuning = 0

min_step = 0.1
xcen_of_view = 960
ycen_of_view = 540
set_pos = {'x': 0.2, 'y': 0., 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}
actual_pos = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}

zed = sl.Camera()

""" ZED PARAMS INITIALIZATION FOR DEPTH SENSING"""
""" ------------------------------------------------------------------------------------------"""
# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
init_params.camera_resolution = sl.RESOLUTION.HD720

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
# Setting the depth confidence parameters
runtime_parameters.confidence_threshold = 100
runtime_parameters.textureness_confidence_threshold = 100


# Create an RGBA sl.Mat object
image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
# Retrieve data in a numpy array with get_data()
image_ocv = image_zed.get_data()

# Create a sl.Mat with float type (32-bit)
depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)

# Create an RGBA sl.Mat object
image_depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

""" ZED PARAMS INITIALIZATION FOR POSITIONAL TRACKING"""
""" ------------------------------------------------------------------------------------------"""
init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                               coordinate_units=sl.UNIT.METER,
                               coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD)

# If applicable, use the SVO given as parameter
# Otherwise use ZED live stream
if len(sys.argv) == 2:
    filepath = sys.argv[1]
    print("Using SVO file: {0}".format(filepath))
    init_params.set_from_svo_file(filepath)

zed = sl.Camera()
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

tracking_params = sl.PositionalTrackingParameters()
zed.enable_positional_tracking(tracking_params)

runtime = sl.RuntimeParameters()
camera_pose = sl.Pose()

camera_info = zed.get_camera_information()
# Create OpenGL viewer
viewer = gl.GLViewer()
viewer.init(camera_info.camera_model)

py_translation = sl.Translation()
pose_data = sl.Transform()

text_translation = ""
text_rotation = ""


def is_pos_reached(set_pos,actual_pos):
    reached = []
    for key in actual_pos:
        #reached.append(set_pos[key] <= actual_pos[key] + 0.1 and set_pos[key] >= actual_pos[key] - 0.1)
        reached.append(actual_pos[key] + 0.3 >= set_pos[key] >= actual_pos[key] - 0.3)
    if all(reached):
        pos_reached = 1
    else:
        pos_reached = 0
    return pos_reached


def next_point(set_pos):
    set_pos['x'] += 0.3
    return set_pos


if perform_mask_tuning:
    lower_bound, upper_bound = mask_bounds()
else:
    lower_bound = np.array([0, 147, 151])
    upper_bound = np.array([255, 170, 170])


while viewer.is_available():    #True:
    """IMAGE AND DEPTH"""
    if zed.grab() == sl.ERROR_CODE.SUCCESS :
        #       NORMAL IMAGE
        # Retrieve the left image in sl.Mat
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        # Use get_data() to get the numpy array
        image_ocv = image_zed.get_data()
        # Display the left image from the numpy array
        #cv.imshow("Image", image_ocv)

        #       DEPTH SENSING - FOR FURTHER PROCESSING
        # Retrieve depth data (32-bit)
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
        # Load depth data into a numpy array
        depth_ocv = depth_zed.get_data()
        # Print the depth value at the center of the image
        #print(depth_ocv[int(len(depth_ocv) / 2)][int(len(depth_ocv[0]) / 2)])

        #       DEPTH SENSING IMAGE - ONLY FOR DISPLAY
        # Retrieve the normalized depth image
        zed.retrieve_image(image_depth_zed, sl.VIEW.DEPTH)
        # Use get_data() to get the numpy array
        image_depth_ocv = image_depth_zed.get_data()
        # Display the depth view from the numpy array
        cv.imshow("Image", np.hstack([image_depth_ocv,image_ocv]))

    """POSITION TRACKING"""
    zed_pose = sl.Pose()
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        tracking_state = zed.get_position(camera_pose)
        if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
            rotation = camera_pose.get_rotation_vector()
            translation = camera_pose.get_translation(py_translation)
            text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
            text_translation = str(
                (round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
            pose_data = camera_pose.pose_data(sl.Transform())
        viewer.updateData(pose_data, text_translation, text_rotation, tracking_state)
        """Additional position tracking"""
        # Get the pose of the camera relative to the world frame
        state = zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        # Display translation and timestamp
        py_translation = sl.Translation()
        tx = round(zed_pose.get_translation(py_translation).get()[0], 2)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 2)
        tz = round(zed_pose.get_translation(py_translation).get()[2], 2)
        #print("Translation: tx: {0}, ty:  {1}, tz:  {2}, timestamp: {3}\n".format(tx, ty, tz, zed_pose.timestamp))
        # Display orientation quaternion
        py_orientation = sl.Orientation()
        ox = round(zed_pose.get_orientation(py_orientation).get()[0], 2)
        oy = round(zed_pose.get_orientation(py_orientation).get()[1], 2)
        oz = round(zed_pose.get_orientation(py_orientation).get()[2], 2)
        ow = round(zed_pose.get_orientation(py_orientation).get()[3], 2)
        #print("Orientation: ox: {0}, oy:  {1}, oz: {2}, ow: {3}\n".format(ox, oy, oz, ow))

        actual_pos['x'] = tx
        actual_pos['y'] = ty
        actual_pos['z'] = tz
        actual_pos['roll'] = ox
        actual_pos['pitch'] = oy
        actual_pos['yaw'] = oz

    if cv.waitKey(1) == ord('q'):
        break

    if do_u_want_zed == 1:
        frame = image_ocv[:, :, 0:3]
        image_depth= image_depth_ocv[:, :, 0:3]
        depth_frame = depth_ocv

    pos_reached = is_pos_reached(set_pos, actual_pos)
    print("actual_pos: {0}     set_pos: {1}\n".format(actual_pos, set_pos))
    time.sleep(1)
    if pos_reached:
        input("press enter for continue")
        set_pos = next_point(set_pos)
    else:
        print("moving")

    orange_mask, orange_mask_in_BGR, enhanced_LAB_into_BGR = masked_img(frame, lower_bound,
                                                                                                  upper_bound)
    contour_detection(orange_mask, enhanced_LAB_into_BGR)

    cv.namedWindow('orange_detection', cv.WINDOW_NORMAL)
    cv.resizeWindow('orange_detection', 1000, 500)
    cv.imshow("orange_detection", np.hstack([enhanced_LAB_into_BGR, orange_mask_in_BGR]))

    # yolo


    img = frame
    img_copy = np.copy(img)

    result = model(img)
    df = result.pandas().xyxy[0]
    if len(df) > 0:
        boxes1 = detections1.add_measurement(df)
        if boxes1 is not None:
            draw_label(img, boxes1)

        boxes2 = detections2.add_measurement(df)
        if boxes2 is not None:
            draw_label(img_copy, boxes2)

    cv.imshow('with nms', img)
    cv.imshow('no nms', img_copy)


    #cv.waitKey(2000)

viewer.exit()
zed.close()