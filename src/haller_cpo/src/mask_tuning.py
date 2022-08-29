import cv2
import numpy as np
#from gate_preprocess import gate_preprocessing
#from countur_detect import contour_detection
#from monitor_view import monitor_capture
#from color_correction import color_corr
from img_preprocess import smoothing, white_balance, mask_morph
import pyzed.sl as sl
#from ZED_video import *


# https://theailearner.com/tag/cv2-inrange-opencv-python/
def nothing(x):
    pass


def mask_bounds():

    do_u_want_zed = 1

    # Open the camera
    # cap = cv2.VideoCapture(0)

    def zed_frame():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            #       NORMAL IMAGE
            # Retrieve the left image in sl.Mat
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            # Use get_data() to get the numpy array
            image_ocv = image_zed.get_data()
            # Display the left image from the numpy array
            # cv.imshow("Image", image_ocv)

            #       DEPTH SENSING - FOR FURTHER PROCESSING
            # Retrieve depth data (32-bit)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            # Load depth data into a numpy array
            depth_ocv = depth_zed.get_data()
            # Print the depth value at the center of the image
            print(depth_ocv[int(len(depth_ocv) / 2)][int(len(depth_ocv[0]) / 2)])

            #       DEPTH SENSING IMAGE - ONLY FOR DISPLAY
            # Retrieve the normalized depth image
            zed.retrieve_image(image_depth_zed, sl.VIEW.DEPTH)
            # Use get_data() to get the numpy array
            image_depth_ocv = image_depth_zed.get_data()
            # Display the depth view from the numpy array
            #cv.imshow("Image", np.hstack([image_depth_ocv, image_ocv]))
            return image_ocv, depth_ocv, image_depth_ocv

    if do_u_want_zed:
        zed = sl.Camera()

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
        image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width,
                           zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
        # Retrieve data in a numpy array with get_data()
        image_ocv = image_zed.get_data()

        # Create a sl.Mat with float type (32-bit)
        depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width,
                           zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)

        # Create an RGBA sl.Mat object
        image_depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width,
                                 zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

    # Create a window
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('lowL', 'image', 0, 255, nothing)
    cv2.createTrackbar('highL', 'image', 255, 255, nothing)

    cv2.createTrackbar('lowA', 'image', 0, 255, nothing)
    cv2.createTrackbar('highA', 'image', 255, 255, nothing)

    cv2.createTrackbar('lowB', 'image', 0, 255, nothing)
    cv2.createTrackbar('highB', 'image', 255, 255, nothing)

    while True:
        # ret, frame = cap.read()
        if do_u_want_zed == 1:
            frame, depth_ocv, image_depth_ocv = zed_frame()
            frame = frame[:, :, 0:3]
        #else:
        #    frame = monitor_capture()

        #   ref = cv2.imread('ref_imgs/ref_img3.png')
        #   frame_color_corr = color_corr(frame, ref)
        frame = smoothing(frame)
        # White balance and transform from RGB -> LAB
        white_bal_LAB = white_balance(frame)

        # get current positions of the trackbars
        ilowL = cv2.getTrackbarPos('lowL', 'image')
        ihighL = cv2.getTrackbarPos('highL', 'image')
        ilowA = cv2.getTrackbarPos('lowA', 'image')
        ihighA = cv2.getTrackbarPos('highA', 'image')
        ilowB = cv2.getTrackbarPos('lowB', 'image')
        ihighB = cv2.getTrackbarPos('highB', 'image')

        # convert color to hsv because it is easy to track colors in this color model
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_lab = np.array([ilowL, ilowA, ilowB])
        higher_lab = np.array([ihighL, ihighA, ihighB])
        # Apply the cv2.inrange method to create a mask
        mask = cv2.inRange(white_bal_LAB, lower_lab, higher_lab)
        # Apply the mask on the image to extract the original color
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('image', frame)

        # orange_mask_morph = mask_morph(mask)
        # cv2.imshow('image_morph', orange_mask_morph)

        # Press q to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # cap.release()
    cv2.destroyAllWindows()
    return lower_lab, higher_lab
#np.savez('mask_bounds', lower_lab=lower_lab, higher_lab=higher_lab)


#lower_bound, upper_bound = mask_bounds()
#print(lower_bound,upper_bound)