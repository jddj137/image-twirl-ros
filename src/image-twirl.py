#!/usr/bin/env python

import cv2 as cv
import sys

from numpy import hstack


class imageTwirl():
    def __init__(self, filepath):
        self.filepath = filepath
        return

    def image_compare(self, img_a, img_b, img_c=None):
        # Stacking the images to print them together for comparison.
        if (img_c is None):
            img_compare = hstack((img_a, img_b))
        else:
            img_compare = hstack((img_a, img_b, img_c))

        return img_compare

    def one_channel_to_three(self, one_chan_img):
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

    def execute_twirl(self, img_in):
        # Load twirl from twirl-config.yaml file.
        file = cv.FileStorage(self.filepath, cv.FileStorage_READ)

        if (not file.isOpened()):
            print('Failed to open ', self.filepath, file=sys.stderr)
            exit(1)

        img_twirl = file.getNode('twirl')

        tmp_img = img_in

        for i in range(img_twirl.size()):
            img_process = img_twirl.at(i).string()
            tmp_img = self.process_image(tmp_img, img_process)

        img_out = tmp_img
        return img_out

    def process_image(self, img_in, img_process):
        """ Set of functions used to process images.

        Below are a variety of useful operations to modify images. When
        combined, these operations form pipelines for pre or post-processing.
        A common image processing pipeline is:
            raw image > convert to gray > blur > canny edge detection
        """

        # Read from twirl-config.yaml file.
        file = cv.FileStorage(self.filepath, cv.FileStorage_READ)

        if (not file.isOpened()):
            print('Failed to open ', self.filepath, file=sys.stderr)
            exit(1)

        # Modify Orientation
        if (img_process == "flip_x"):
            # 0, for flipping the image around the x-axis (vertical flipping).
            img_out = cv.flip(img_in, 0)

        elif (img_process == "flip_y"):
            # > 0 for flipping around the y-axis (horizontal flipping).
            img_out = cv.flip(img_in, 1)

        elif (img_process == "flip_xy"):
            # < 0 for flipping around both axes.
            img_out = cv.flip(img_in, -1)

        # Modify Color
        elif (img_process == "bgr2hsv"):
            # Switch to hue, saturation, value color mode.
            # Sometimes better for color tracking.
            img_out = cv.cvtColor(img_in, cv.COLOR_BGR2HSV)

        elif (img_process == "bgr2gray"):
            # Converting the image to grayscale.
            gray = cv.cvtColor(img_in, cv.COLOR_BGR2GRAY)
            img_out = self.one_channel_to_three(gray)

        # Image Smoothing
        elif (img_process == "blur"):
            # Smoothing that softens edges.
            kSize = file.getNode('blur.kernalSize')
            (w, h) = int(kSize.at(0).real()), int(kSize.at(1).real())

            img_out = cv.blur(img_in, (w, h))

        elif (img_process == "gaussianBlur"):
            # Gaussian filtering is highly effective in removing noise.
            kSize = file.getNode('gaussianBlur.kernalSize')
            (w, h) = int(kSize.at(0).real()), int(kSize.at(1).real())

            sigmaX = file.getNode('gaussianBlur.sigmaX').real()
            sigmaY = file.getNode('gaussianBlur.sigmaY').real()

            img_out = cv.GaussianBlur(img_in, (w, h), sigmaX, sigmaY)

        elif (img_process == "bilateralFilter"):
            # Smoothing without removing edges.
            diameter = int(file.getNode('bilateralFilter.diameter').real())
            sigmaClr = file.getNode('bilateralFilter.sigmaColor').real()
            sigmaSpc = file.getNode('bilateralFilter.sigmaSpace').real()

            img_out = cv.bilateralFilter(img_in, diameter, sigmaClr, sigmaSpc)

        else:
            print("Twirl operation not found.")
            img_out = img_in

        return img_out


def test_twirl():
    filepath = "/Users/josh/repos/image_twirl_ros/launch/twirl-config.yaml"
    img = "/Users/josh/repos/image_twirl_ros/test/rick-morty.jpg"
    twirler = imageTwirl(filepath)
    img_in = cv.imread(img)
    img_twirl = twirler.execute_twirl(img_in)

    cv.imshow('Image Twirl', img_twirl)

    k = cv.waitKey(0)
    if k == 27:         # wait for ESC key to exit
        cv.destroyAllWindows()


def test_twirl_config_update():
    while(1):
        filepath = "/Users/josh/repos/image_twirl_ros/launch/twirl-config.yaml"
        img = "/Users/josh/repos/image_twirl_ros/test/rick-morty.jpg"
        twirler = imageTwirl(filepath)
        img_in = cv.imread(img)
        img_twirl = twirler.execute_twirl(img_in)

        cv.imshow('Image Twirl', img_twirl)

        k = cv.waitKey(1000)
        if k == 27:         # wait for ESC key to exit
            cv.destroyAllWindows()
            break


if __name__ == "__main__":
    test_twirl()
    test_twirl_config_update()
