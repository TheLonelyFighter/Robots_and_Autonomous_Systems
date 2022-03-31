#! /usr/bin/python
import cv2 
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Image

from std_msgs.msg import Empty
#from djitellopy import Tello
from geometry_msgs.msg import Twist
import time
from nav_msgs.msg import Odometry



class CamSubscriber(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.cnt = 0
        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)
        self.publisher_twist = self.create_publisher(Twist, '/control', 10)
        self.publisher_take_off = self.create_publisher(Empty, '/takeoff', 10)
        self.publisher_land = self.create_publisher(Empty, '/land', 10)
        self.moved = False
        self.TAKE_OFF = False
        self.GOOD_HEIGHT = False
        self.CENTERED = False
        self.PASSED = False
        self.height = 720



    def image_sub_callback(self, msg):

        # Convert ROS Image message to OpenCV2
        cv2_img = self.imgmsg_to_cv2(msg)
        #print("Received image msg")
        self.cnt += 1
        #cv.imwrite('camera_image_tello{}.jpeg'.format(self.cnt), cv2_img)

        if (self.cnt % 10 == 0):
            self.move_control(cv2_img)




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

    def move_control(self,img):

      

        center  = self.process_image(img)
        cv2.imwrite("real_time_img.png", img)
        print(center)
        if (self.TAKE_OFF == False):            
            print("Taking off")
            empty_msg = Empty()
            self.publisher_take_off.publish(empty_msg)
            time.sleep(2) #wait, otherwise it oublishes more than 4 take off cmds and the tello driver crashes 
            self.TAKE_OFF = True

        
            

        if (self.GOOD_HEIGHT == False):
            self.move(0.0,0.0,40.0, 3 )
            self.GOOD_HEIGHT = True
            self.CENTERED = True

        
        if (self.CENTERED == True):
            
            upper_bound = self.height / 2 - 50
            lower_bound = self.height / 2 + 50
            if center[1] < upper_bound:
                print('UP')
                self.move(0.0,0.0,10.0, 0.1)
                pass
            elif center[1] > lower_bound:
                print("DOWN")
                self.move(0.0,0.0,-10.0, 0.1)
                pass
            else:
                print("In center")
                self.move(0.0,40.0,0.0, 4)
                self.PASSED = True
                self.CENTERED = False
                pass
        else:
            

            if (self.PASSED == True):
                self.move(0.0,0.0,0.0, 0.01)
                empty_msg = Empty()
                self.publisher_land.publish(empty_msg)
                self.LANDED = True
               
        
        # if (self.LANDED):
        #     something = 1


        #self.move(0.0,0.0,0.0, 0.01 )

        

        


    def process_image(self, img):

        #generate HSV image
        img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #lower and upper range for masking
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])

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

        #Draw contour around the one with maximum area. The index of the contour with maxiumum area is assigned to max_cnt_index in previous bloack of code
        cv2.drawContours(img_result, cnts, max_cnt_index, (0,255,0), 3)

        #draw rotated rectangle using minAreaRect
        rect = cv2.minAreaRect(cnts[max_cnt_index])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img_result,[box],0,(0,0,255),3)

        #find the center of the rectangle and draw it on the image
        center = (int(rect[0][0]), int(rect[0][1]))
        cv2.circle(img_result, center, 10,(0,0,255),3)

        cv2.imwrite("result.png", img_result)

        
        return center
        
        
        
        

    def move(self,speed_x, speed_y, speed_z, seconds = 0): #move forward for n seconds
        msg = Twist()
        msg.linear.x = speed_x
        msg.linear.y = speed_y
        msg.linear.z = speed_z

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.publisher_twist.publish(msg)
        print("moving with speed x,y,z", speed_x,speed_y,speed_z)    
        time.sleep(seconds)    
        
    def spin(self, speed):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = speed

        self.publisher_twist.publish(msg)
        print("spinning")        



def main():
    rclpy.init()
    cam_subscriber = CamSubscriber()

    # Spin until ctrl + c
    print("Moving node initialized")
    rclpy.spin(cam_subscriber)

    cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()