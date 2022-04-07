#! /usr/bin/python

import base64
import cv2
import numpy as np
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CamPublisher(Node):

    def __init__(self):
        super().__init__('image_pub')
        self.processed_img_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)
        self.NO_GATE_DETECTED = True
        self.FULL_RECTANGLE_DETECTED = False
        self.OPEN_RECTANGLE_DETECTED = False

        self.cnt = 0

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

    def image_sub_callback(self, msg):

        # Convert ROS Image message to OpenCV2
        cv2_img = self.imgmsg_to_cv2(msg)
        #print("Received image msg")
        self.cnt += 1
        #cv.imwrite('camera_image_tello{}.jpeg'.format(self.cnt), cv2_img)

        if (self.cnt % 1 == 0):
            center, processed_img = self.process_image(cv2_img)
            imgMsg = self.cv2_to_imgmsg(processed_img, "bgr8")

            self.processed_img_publisher.publish(imgMsg)
    
    def process_image(self, img):

        #generate HSV image
        img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #lower and upper range for masking
        # lower_red = np.array([0,50,50]) #this is for RED
        # upper_red = np.array([10,255,255])

        lower_red = np.array([35,45,100])
        upper_red = np.array([65,155,255])

        #generate the mask
        mask = cv2.inRange(img_hsv, lower_red, upper_red)

        #extract the masked image from the original image
        img_result = cv2.bitwise_and(img, img, mask=mask)

        #genrate greyscale version for finding contours
        gray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)

        #find contours
        cnts, hierachy = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        #find contour with maximum area
        max_area = -1
        max_cnt_index = 0
        for i in range(len(cnts)):
            area = cv2.contourArea(cnts[i])
            if area>max_area:
                max_cnt_index = i
                max_area = area

        print("Max area:", max_area)

        if max_area > 40000:
            self.FULL_RECTANGLE_DETECTED = True
            self.NO_GATE_DETECTED = False
        elif max_area > 4000 and max_area < 40000:
            self.OPEN_RECTANGLE_DETECTED = True
            self.NO_GATE_DETECTED = False
        else:
            self.NO_GATE_DETECTED = True
            return None

        if self.FULL_RECTANGLE_DETECTED == True:
            #Draw contour around the one with maximum area. The index of the contour with maxiumum area is assigned to max_cnt_index in previous block of code
            cv2.drawContours(img_result, cnts, max_cnt_index, (0,255,0), 3)

            #draw rotated rectangle using minAreaRect
            rect = cv2.minAreaRect(cnts[max_cnt_index])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img_result,[box],0,(0,0,255),3)

            #find the center of the rectangle and draw it on the image
            center = (int(rect[0][0]), int(rect[0][1]))
            cv2.circle(img_result, center, 10,(0,0,255),3)


        elif self.OPEN_RECTANGLE_DETECTED == True:
            #find contour with maximum area
            areas = {}
            max_area = -1
            max_cnt_index = 0
            for i in range(len(cnts)):
                area = cv2.contourArea(cnts[i])
                areas[i] = area
                
            #extract only 4 contours with max area
            sort_areas = sorted(areas.items(), key=lambda x: x[1], reverse=True)[:4]

            #get keys for maximum area contours
            keys = []
            for item in sort_areas:
                keys.append(item[0])

            #min x and y, max x and y for rect to draw around the 4 contours
            min_x,min_y = 2000,20000
            max_x, max_y = 0 , 0

            #iterate through each contour and find min and max values
            for key in keys:
                (x,y,w,h) = cv2.boundingRect(cnts[key])
                min_x, max_x = min(x, min_x), max(x+w, max_x)
                min_y, max_y = min(y, min_y), max(y+h, max_y)

            #draw rect around the contours
            if max_x - min_x > 0 and max_y - min_y > 0:
                rect = cv2.rectangle(img_result, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)
                center = ((min_x + max_x) / 2, (min_y + max_y) / 2)

        # cv2.circle(img_result, (480,360),20,color=(0, 255, 0), thickness=2  ) # plot image center
        # cv2.circle(img_result, (int(center[0]), int(center[1])),10,color=(0, 0, 255), thickness=2  ) # plot gate center
        #cv2.imwrite("square_mask.png", img_result)
        return center, img_result

def main(args=None):
    rclpy.init()
    cam_publisher = CamPublisher()
    rclpy.spin(cam_publisher)

    cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()