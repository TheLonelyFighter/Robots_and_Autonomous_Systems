#! /usr/bin/python

import base64
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class CamPublisher(Node):

    def __init__(self):
        super().__init__('image_pub')
        self.optitrack_pub = self.create_publisher(PoseStamped, 'optitrack', 10)

        self.cnt = 0
        self.update_optitrack()

    def update_optitrack(self):
        goal = PoseStamped()

        goal.header.frame_id = "map"

        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        self.optitrack_pub.publish(goal)

def main(args=None):
    rclpy.init()
    cam_publisher = CamPublisher()
    rclpy.spin(cam_publisher)

    cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()