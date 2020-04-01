#!/usr/bin/env python

import cv2 as cv
import ros_tutorials.srv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from numpy import hstack
from sensor_msgs.msg import Image


class imagePipeline():

    def __init__(self):
        """ Creates an instance of the ROS node.

        Generally the Python init() function initializes the node in ROS
        using rospy.init_node() and sets up the necessary node elements,
        such as:
            - publisher/subscriber
            - services (request/response)
            - callback functions
            - image conversion (for nodes using images)
        The init() may also contain class variables, local variables, or
        other logic.
        """

        ''' ROS Node Elements '''
        rospy.init_node('image_pipeline')

        # Setup the publisher and subscriber.
        # TODO: Add the "publisher" to output processed images.
        # Hint: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29



        # TODO: Complete the "subscriber" to camera input.
        # Hint: Try the terminal commands 'rostopic list' and 'rostopic type [topic]' with camera running.



        # Create Service (Server side)
        rospy.Service(
            '/set_process_type', ros_tutorials.srv.SendString,
            self.set_process_type_handle)

        # Converts between ROS Image messages and OpenCV images.
        self.bridge = CvBridge()

        # Specifies the rate (Hz) of publishers and other rospy processes.
        rospy.Rate(10)

        ''' Class Variables '''
        self.process_type = "raw"  # Default process_type allows passthrough

        # TODO (optional): Add a rospy info message notifying user that the
        # node is initialized.


    def set_process_type_handle(self, request):
        """ Callback invoked when new service requests are received.

        The handler is invoked with the service request message and should
        return the appropriate service response message. If the handler returns
        None or raises an exception, an error will be passed to the client.

        Arguments:
            request {[type]} -- Parameter names found in the .srv file

        Returns:
            Handler may return ServiceResponse, single-argument,
            tuple, list, or dict
        """
        if request.data not in PROCESSING_OPTIONS:
            # TODO (optional): Add a periodic rospy warning every 5 seconds
            # notifying user that an image processing type was not selected.
            # Hint: http://wiki.ros.org/rospy/Overview/Logging



            self.process_type = "raw"

            # TODO: Send a fail response. See the parameter names in SendString.srv.
            # Hint: http://wiki.ros.org/rospy/Overview/Services#Providing_services

        else:
            # TODO (optional): Add a periodic rospy info message every
            # 5 seconds notifying user that an image processing type has not
            # been selected.


            self.process_type = request.data

            # Send an affirmative response.
            return {'success': True}

    def camera_callback(self, img_data):
        """ Function to call when data is received from the camera image topic.

        ROS callback functions are generally message handlers assigned to topic
        subscriptions. When a message arrives, ROS passes the new message data
        to the callback function.

        Callback functions are defined by the user, but not called by the user.
        They can do something or nothing with the data, and may set global
        variables or act on local variables.

        Note: Callback functions can be tricky because they are asynchronous
        with other pieces of the code and are called at the rate which data
        from the publisher is received. Use caution if another piece of code
        relies on the callback data.

        Arguments:
            data {[type]} -- This refers to the data/Message type being
                            received by the subscription from a publisher.
        """
        try:
            # Converts ROS image message to OpenCV image.
            img_raw = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError as e:
            # TODO (optional): Add a rospy error message to log
            # the CvBridge exception.

            return

        final_img = process_image(img_raw, self.process_type)

        try:
            final_img = self.bridge.cv2_to_imgmsg(final_img, encoding="bgr8")

            # TODO: Publish the processed image.

        except CvBridgeError as e:
            # TODO (optional): Add a rospy error message to log
            # the CvBridge exception.

            return


""" Utility Functions for an Image Processing Class

These could technically be placed in a seperate file for a larger project.
"""
PROCESSING_OPTIONS = {"raw", "flip", "color", "blur", "edge", "your-name-here"}


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

    ''' Helpful References '''
    # https://docs.opencv.org/trunk/d2/d96/tutorial_py_table_of_contents_imgproc.html
    # https://docs.opencv.org/master/d8/d01/group__imgproc__color__conversions.html
    # https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
    # https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
    # https://www.pyimagesearch.com/2014/09/08/thresholding-simple-image-segmentation-using-opencv/


if __name__ == '__main__':
    node = imagePipeline()
    try:
        rospy.spin()
    except KeyboardInterrupt:  # Press Ctrl+c on keyboard to exit.
        print("Shutting down...")
