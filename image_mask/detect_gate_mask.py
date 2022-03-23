#! /usr/bin/python
import base64
import cv2 as cv
import numpy as np
import imutils

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Image


class CamSubscriber(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.cnt = 0
        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)


    def image_sub_callback(self, msg):

        # Convert ROS Image message to OpenCV2
        cv2_img = self.imgmsg_to_cv2(msg)
        rgb_img = cv.cvtColor(cv2_img, cv.COLOR_BGR2RGB)
        self.cnt += 1
        # Save OpenCV2 image as a jpeg
        #cv.imwrite('raw_images/camera_image_{}.jpeg'.format(self.cnt), cv2_img)
        #print("Received an image!_{}".format(self.cnt))
        self.process_image(rgb_img)


    def imgmsg_to_cv2(self, img_msg):
        n_channels = len(img_msg.data) // (img_msg.height * img_msg.width)
        dtype = np.uint8

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width),
                            dtype=dtype, buffer=img_buf)
        else:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                            dtype=dtype, buffer=img_buf)

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            cv2_img = cv2_img.byteswap().newbyteorder()

        return cv2_img

    def process_image(self, image):
        
        #Make color mask
        #Modify values here to better detect the color of the gate
        # lower_boundary = [185, 0, 0]
        # upper_boundary = [245, 150, 90]
        lower_boundary = [180, 0, 0]
        upper_boundary = [255, 170, 170]
        lower = np.array(lower_boundary, dtype = "uint8")
        upper = np.array(upper_boundary, dtype = "uint8")
        mask = cv.inRange(image, lower, upper)
        output = cv.bitwise_and(image, image, mask = mask)

        #Detect contours
        #resized = imutils.resize(output, width=300)
        gray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
        cnts = cv.findContours(gray.copy(), cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)


        #Find the contour with the biggest area
        max_area = -1

        for index, contour in enumerate(cnts):
            area = cv.contourArea(contour)
            if area > max_area:
                max_contour_index = index
                max_area = area

        #Draw only the contour with the biggest area
        cv.drawContours(image, cnts, max_contour_index, (0,255,0), 3)

        #Approximate the contour with a polygon, does not really work, maybe adjust epsilon
        #check here: https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        #maybe switch to convex hull approximation
        epsilon = 0.1*cv.arcLength(cnts[max_contour_index],True)
        approx = cv.approxPolyDP(cnts[max_contour_index],epsilon,True)    
        
        start = approx[0].tolist()
        stop = approx[1].tolist()
        print(start[0][0])
        image = cv.rectangle(image, (start[0][0],start[0][1]), (stop[0][0],stop[0][1]), (0,0,0), -1)

        

        print("Saving image ", self.cnt)
        cv.imwrite('processed_images/processed_image_{}.jpeg'.format(self.cnt), image)

        # if len(cnts) >= 1:            
        #     print('Contour found, saving image')
        #     cv.imwrite('processed_images/contour_image_{}.jpeg'.format(self.cnt), image)


def main():
    rclpy.init()
    cam_subscriber = CamSubscriber()
    print("cam_sub init")
    # Spin until ctrl + c
    rclpy.spin(cam_subscriber)

    cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
