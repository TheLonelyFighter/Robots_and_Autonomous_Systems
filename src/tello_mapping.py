#! /usr/bin/python
from ast import While
import base64
import cv2 as cv
import numpy as np
import imutils

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
import time
from geometry_msgs.msg import Twist, PoseStamped



class CamSubscriber(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.cnt = 0
        self.image1=""
        self.pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/tello_5FC349/pose', self.pose_sub_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)
        self.take_pic = self.create_subscription(String, '/take_pic', self.take_pic_callback, 10)
        #self.take_off = self.create_subscription(String, '/take_pic', self.take_pic_callback, 10)
        self.publisher_twist = self.create_publisher(Twist, '/control', 10)
        self.publisher_take_off = self.create_publisher(Empty, '/takeoff', 10)
        self.publisher_land = self.create_publisher(Empty, '/land', 10)
        self.publisher_map = self.create_publisher(Empty, '/land', 10)
        self.image_pub = self.create_publisher(Image, '/tello_map_image', 10)
        self.TAKE_PIC = False
       
        self.Landed = False
        self.TAKE_OFF = False
        self.GOOD_HEIGHT = False
        self.z_position = 0
        self.image_message = []

        self.run()

        


    def pose_sub_callback(self, msg):
        #print(msg)
        self.z_position = msg.pose.position.z
        #print(self.z_position)
        if (2 < msg.pose.position.z <= 2.5):
            self.GOOD_HEIGHT = True
            print("Good height achieved")
            #self.take_pic1()

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
	
    def take_pic_callback(self, msg):
        print("taking photo")
        cv.imwrite( "photo.jpg", self.image1)
    
    def take_pic1(self):
        print("Taking picture")
        cv.imwrite( "photo.jpg", self.image1)
        print("Publishing image")
        self.image_pub.publish(self.image_message)

    def image_sub_callback(self, msg):

        #imgMsg = self.cv2_to_imgmsg(image, "bgr8")
        self.image_message = msg
        
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

        self.run()

        return cv2_img

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

    def run(self):

        
        if (self.TAKE_OFF == False):            
            print("Taking off")
            empty_msg = Empty()
            self.publisher_take_off.publish(empty_msg)
            time.sleep(5) #wait, otherwise it oublishes more than 4 take off cmds and the tello driver crashes   
            self.TAKE_OFF = True

        if (self.GOOD_HEIGHT == False):  # move up
            print("Moving up")
            self.move(0.0,0.0,20.0, 0)
            #self.GOOD_HEIGHT = True
            
        

        if (self.GOOD_HEIGHT == True and self.Landed == False):
            
            self.move(0.0,0.0,0.0, 0)
            #self.take_pic()
            time.sleep(1)
            if (self.TAKE_PIC == False):
                print("taking picture")
                #If you only publish ONE picture, the topic does not receive the image;
                self.take_pic1()
                self.take_pic1()
                self.take_pic1()
                #self.TAKE_PIC = True
            print(self.z_position)
            
            time.sleep(2)

            empty_msg = Empty()
            self.publisher_land.publish(empty_msg)
            time.sleep(2)
            self.Landed = True

    def process_image(self, image):
        
        
        self.image1 = image
       
        
        

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
