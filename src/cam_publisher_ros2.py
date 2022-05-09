#! /usr/bin/python

import base64
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CamPublisher(Node):

    def __init__(self):
        super().__init__('image_pub')
        self.image_pub = self.create_publisher(Image, '/tello_map_image', 10)

        self.cnt = 0
        self.read_image()

    def read_image(self):
        frame = cv2.imread("../images/obstacle_course/test1.jpg")
        imgMsg = self.cv2_to_imgmsg(frame, "bgr8")

        self.image_pub.publish(imgMsg)

    def cv2_to_imgmsg(self, cv2_img, encoding = "passthrough"):
        img_msg = Image()
        img_msg.height = cv2_img.shape[0]
        img_msg.width = cv2_img.shape[1]
        img_msg.encoding = encoding

        if cv2_img.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cv2_img.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg

def main(args=None):
    rclpy.init()
    cam_publisher = CamPublisher()
    rclpy.spin(cam_publisher)

    cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()