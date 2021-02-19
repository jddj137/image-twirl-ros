#!/usr/bin/env python

import cv2 as cv
import image_twirl_ros.srv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from numpy import hstack
from sensor_msgs.msg import Image


class imageTwirl():
    def __init__(self):
        self.pipeline = {"raw", "flip", "color", "blur", "edge", "your-name-here"}

        return

    def image_compare(img_a, img_b, img_c=None):
        # Stacking the images to print them together for comparison.
        if (img_c is None):
            img_compare = hstack((img_a, img_b))
        else:
            img_compare = hstack((img_a, img_b, img_c))

        return img_compare

    def one_channel_to_three(one_chan_img):
        """ Transforms an image with one channel into image with three channels.

        When OpenCV performs a transform to gray scale or edge detection,
        the three color channels (RGB) are reduced into a single channel.
        This causes an error when trying to convert back to "bgr8" with the
        cv_bridge. The cv.merge() function overlays the single channel onto
        itself three times, effectively creating a three channel image.

        Arguments:
            one_chan_img {OpenCV Image} -- Image with one channel

        Returns:
            OpenCV Image -- Image with three channels
        """
        three_chan_img = cv.merge((one_chan_img, one_chan_img, one_chan_img))

        return three_chan_img


def process_image(img_raw, process_type):
    """ Set of functions used to process images.

    Below are a variety of useful operations to modify images. When
    combined, these operations form pipelines for pre or post-processing.
    A common image processing pipeline is:
        raw image > convert to gray > blur > canny edge detection
    """

    # Modify Orientation
    if (process_type == "flip"):
        # 0, for flipping the image around the x-axis (vertical flipping).
        flipVertical = cv.flip(img_raw, 0)
        # > 0 for flipping around the y-axis (horizontal flipping).
        flipHorizontal = cv.flip(img_raw, 1)
        # < 0 for flipping around both axes.
        flipBoth = cv.flip(img_raw, -1)

        final_img = image_compare(flipVertical, flipHorizontal, flipBoth)

    # Modify Color
    elif (process_type == "color"):
        # Switch to hue, saturation, value color mode.
        # Sometimes better for color tracking.
        hsv = cv.cvtColor(img_raw, cv.COLOR_BGR2HSV)

        # Converting the image to grayscale.
        gray = cv.cvtColor(img_raw, cv.COLOR_BGR2GRAY)
        gray3 = one_channel_to_three(gray)

        final_img = image_compare(img_raw, hsv, gray3)

    # Image Smoothing
    elif (process_type == "blur"):
        # Smoothing that softens edges.
        blur = cv.blur(img_raw, (5, 5))

        # Gaussian filtering is highly effective in removing noise from image.
        gblur = cv.GaussianBlur(img_raw, (25, 25), 0)

        # Smoothing without removing edges.
        img_filter = cv.bilateralFilter(img_raw, 5, 50, 50)

        final_img = image_compare(blur, gblur, img_filter)

    # Edge Detection
    elif (process_type == "edge"):
        # Using the canny filter to get edges.
        edges = cv.Canny(img_raw, 45, 50)
        edges3 = one_channel_to_three(edges)

        # Using the canny filter with different params for higher sensitivity.
        edges_high_thresh = cv.Canny(img_raw, 100, 200)
        edges_high_thresh3 = one_channel_to_three(edges_high_thresh)

        final_img = image_compare(img_raw, edges3, edges_high_thresh3)

    # TODO: Extra credit. Make your own image processing pipeline.
    elif (process_type == ["your name here"]):
        img_a = 0  # placehold for a real cv function
        img_b = 0  # placehold for a real cv function
        img_c = 0  # placehold for a real cv function

        final_img = image_compare(img_a, img_b, img_c)

    else:
        final_img = img_raw

    return final_img