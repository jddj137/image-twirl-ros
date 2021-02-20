#!/usr/bin/env python

import rospy
import imageTwirl

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class imageTwirlRos():

    def __init__(self):
        """ Creates an instance of the ROS node.
        """
        self.twirl_config_filepath = rospy.get_param('~twirl_config_filepath')
        self.twirler = imageTwirl(self.twirl_config_filepath)

        rospy.init_node('image_twirl')

        # Setup the publisher and subscriber.
        sub_topic = rospy.get_param('~image_input_topic')
        pub_topic = rospy.get_param('~image_output_topic')

        self.img_sub = rospy.Subscriber(
            sub_topic, Image, self.camera_callback)

        self.img_pub = rospy.Publisher(
            pub_topic, Image, queue_size=3, latch=True)

        # Converts between ROS Image messages and OpenCV images.
        self.bridge = CvBridge()

        # Specifies the rate (Hz) of publishers and other rospy processes.
        rospy.Rate(10)

        rospy.loginfo_once("The image_twirl node is initialized.")

    def camera_callback(self, img_data):
        """ Function called when data is received from the camera image topic.

        Arguments:
            data {[type]} -- This refers to the data/Message type being
                            received by the subscription from a publisher.
        """
        try:
            # Converts ROS image message to OpenCV image.
            img_raw = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        final_img = self.twirler.execute_twirl(img_raw)

        try:
            final_img = self.bridge.cv2_to_imgmsg(final_img, encoding="bgr8")

            self.img_pub.publish(final_img)
        except CvBridgeError as e:
            rospy.logerr(e)
            return


if __name__ == '__main__':
    node = imageTwirlRos()
    try:
        rospy.spin()
    except KeyboardInterrupt:  # Press Ctrl+c on keyboard to exit.
        print("Shutting down...")
