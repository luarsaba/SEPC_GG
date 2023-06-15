#!/usr/bin/env python3
import os
import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

TOPIC_NAME = '/mitchy/duckdet/image/compressed'

class DuckDet (DTROS):
    def __init__(self, node_name):
        super(DuckDet, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            TOPIC_NAME,
            CompressedImage,
            self.callback,
            queue_size =1
        )

        self.duck_detected_pub = rospy.Publisher(
            'duck_detected',
            Bool,
            queue_size=1
        )

    def callback(self, msg):
        print(f'callback with type ${type(msg)}')

        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV
        hsv_img = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2HSV)

        # lower range of red color in HSV
        lower_range = (16, 50, 50)
        # upper range of red color in HSV
        upper_range = (21, 255, 255)

        mask = cv2.inRange(hsv_img, lower_range, upper_range)
        color_image = cv2.bitwise_and(img_cv2, img_cv2, mask=mask)


        roi = color_image[int(color_image.shape * 0.4):,:]

        if cv2.countNonZero(roi) > 0:
            self.duck_detected_pub.publisher(True)
        else:
            self.duck_detected_pub.publisher(False)
if __name__ == '__main__':
    node = DuckDet(node_name='duckdet')
    rospy.spin()
