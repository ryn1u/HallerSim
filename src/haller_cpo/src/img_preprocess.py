import cv2 as cv
import os
import numpy as np
from mss import mss
from PIL import Image

from skimage import exposure

def smoothing(frame):
    avg_blurr = cv.blur(frame, (5, 5))
    gaus_blurr = cv.GaussianBlur(avg_blurr, (5, 5), 0)
    med_blurr = cv.medianBlur(gaus_blurr, 9)
    return med_blurr


# From https://stackoverflow.com/questions/46390779/automatic-white-balancing-with-grayworld-assumption/46391574
def white_balance(LAB_frame):
    result = cv.cvtColor(LAB_frame, cv.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    return result

"""
# Return orange mask - NEED TUNING
def orange_mask(LAB_frame):
    lower_bound = np.array([0, 128, 126])
    upper_bound = np.array([255, 255, 255])
    orange_mask_frame = cv.inRange(LAB_frame, lower_bound, upper_bound)
    return orange_mask_frame
"""

def mask_morph(mask):
    kernel = np.ones((9,9), np.uint8)
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    dilation = cv.dilate(closing, kernel, iterations=1)
    return dilation


def masked_img(frame, lower_bound, upper_bound):
    #ref = cv.imread('ref_imgs/ref_img3.png')
    #frame_color_corr = color_corr(frame, ref)
    frame = smoothing(frame)
    #cv.imshow("Smoothed image", frame)
    # White balance and transform from RGB -> LAB
    white_bal_LAB = white_balance(frame)
    #cv.imshow("White balance", white_bal_LAB)

    masked_frame = cv.inRange(white_bal_LAB, lower_bound, upper_bound)
    #cv.imshow("Binary image before morphological operations", masked_frame)
    masked_frame_morph = mask_morph(masked_frame)

    # convert binary img to BGR just to show with BGR in one window
    masked_frame_in_rgb = cv.cvtColor(masked_frame_morph, cv.COLOR_GRAY2BGR)
    # convert back LAB img into BGR
    converted_BGR = cv.cvtColor(white_bal_LAB, cv.COLOR_LAB2BGR)

    return masked_frame_morph, masked_frame_in_rgb, converted_BGR


def color_corr(src,ref):
    # load the source and reference images
    #print("[INFO] loading source and reference images...")
    #src = monitor_capture()
    #ref = cv2.imread('ref_img.png')
    # determine if we are performing multichannel histogram matching
    # and then perform histogram matching itself
    #print("[INFO] performing histogram matching...")
    multi = True  # if src.shape[-1] > 1 else False
    matched = exposure.match_histograms(src, ref, multichannel=multi)
    # show the output images
    #cv2.imshow("Source", src)
    #cv2.imshow("Reference", ref)
    #cv2.imshow("Matched", matched)
    #cv2.waitKey(0)
    return src

"""
# construct a figure to display the histogram plots for each channel
# before and after histogram matching was applied
(fig, axs) =  plt.subplots(nrows=3, ncols=3, figsize=(8, 8))
# loop over our source image, reference image, and output matched
# image
for (i, image) in enumerate((src, ref, matched)):
	# convert the image from BGR to RGB channel ordering
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	# loop over the names of the channels in RGB order
	for (j, color) in enumerate(("red", "green", "blue")):
		# compute a histogram for the current channel and plot it
		(hist, bins) = exposure.histogram(image[..., j],
			source_range="dtype")
		axs[j, i].plot(bins, hist / hist.max())
		# compute the cumulative distribution function for the
		# current channel and plot it
		(cdf, bins) = exposure.cumulative_distribution(image[..., j])
		axs[j, i].plot(bins, cdf)
		# set the y-axis label of the current plot to be the name
		# of the current color channel
		axs[j, 0].set_ylabel(color)

	# set the axes titles
	axs[0, 0].set_title("Source")
	axs[0, 1].set_title("Reference")
	axs[0, 2].set_title("Matched")
	# display the output plots
	plt.tight_layout()
	plt.show()	"""


def contour_detection(mask_vertical_bar, frame):
    (contours, _) = cv.findContours(mask_vertical_bar, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    box_points = list()
    areas = [cv.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    contour = contours[max_index]
    x, y, w, h = cv.boundingRect(contour)
    b_box = [x, y, w, h]
    #cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return b_box


    # DEBUGGING:
    """if len(box_points) > 3:
        print("BREAKPOINT")
        print(box_points)
        for box in box_points:
            cv.drawContours(frame, [box], 0, (0, 0, 255), 10)
        cv.imshow("frame", frame)
        cv.waitKey(0)"""


def contour_filter(contour, rect, box):

    # remove contours that are too small
    perimeter = cv.arcLength(contour, True)
    if perimeter < 60:
        return False

    # remove contours that are too small again
    area = cv.contourArea(contour)
    if area < 2000:
        return False

    # Remove 'boxy' contours - keep rod-like ones
    # src says this is fast way to calc distance betw two points.
    # src: https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
    # Since contour's 0-index is the lowest (greatest y-val) point, and goes clockwise, 0-1 and 1-2 must refer
    # to height and width (in any order)
    sides = [np.linalg.norm(box[0] - box[1]), np.linalg.norm(box[1] - box[2])]
    height = np.amax(sides)
    width = np.amin(sides)
    aspect_ratio = height / width
    if aspect_ratio < 1:
        return False


    # Our rods should fit tightly inside its bounding box. If not, then its probably distorted surface reflection
    # Solidity can be thought of as an approximation of the fit
    hull = cv.convexHull(contour)
    hull_area = cv.contourArea(hull)
    solidity = float(area) / hull_area
    if solidity < 0.8:
        return False

    # We want our rod to be vertical i.e. long side should be vertical.
    # src says angle in rect[2] (from minAreaRect)is calculated from first edge counterclockwise from horizontal (edge between box[0] and box[3]).
    # src: https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/
    transformed_angle = calc_minAreaRect_orientation(rect, sides)
    if transformed_angle < -135 or transformed_angle > -45:
        return False

    return True


# Transforms the [-90, 0] range of rect[2] to [-180, 0]
# designate 'height' as edge between box[0] and box[1]
# 'width' as edge between box[1] and box[2]
def calc_minAreaRect_orientation(rect, sides):
    if sides[1] < sides[0]:
        return rect[2] - 90
    else:
        return rect[2]

