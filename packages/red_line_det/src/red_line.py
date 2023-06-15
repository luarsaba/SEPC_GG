#!/usr/bin/env python3
import os
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

TOPIC_NAME = '/mitchy/red_line/image/compressed'


class RedLineEdge(DTROS):
    def __init__(self, node_name):

        super(RedLineEdge, self).__init__(node_name=node_name,
                                          node_type=NodeType.PERCEPTION)

        # for converting from CompressedImage to cv2 and vice versa
        self.bridge = CvBridge()

        # subscribing to topic TOPIC_NAME
        self.sub = rospy.Subscriber(TOPIC_NAME,
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="10MB"
                                    )

        # publisher to signal if red is detected
        self.red_detected_pub = rospy.Publisher(
                                'red_detected',
                                Bool,
                                queue_size=1
                                )

    def callback(self, msg):
        print(f'callback with type ${type(msg)}')

        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV
        hsv_img = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create a mask for red pixels
        red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Define the region of interest (ROI) as the lower part of the image
        height, width = red_mask.shape
        roi = red_mask[int(height * 0.4):, :]  # Choose the lower 30% of the image

        # Check if there are any red pixels in the ROI
        if cv2.countNonZero(roi) > 0:
            self.red_detected_pub.publish(True)
        else:
            self.red_detected_pub.publish(False)


if __name__ == '__main__':
    node = RedLineEdge(node_name='red_line')
    rospy.spin()
