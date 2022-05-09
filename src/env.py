"""
Env 2D
@author: huiming zhou
"""

import cv2
from create_coordinate_lists import get_coordinates
from move_drone import imgmsg_to_cv2

class Env:
    def __init__(self):
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        

    def obs_map(self, image):
        """
        Takes an image matrix and rescales it, then searches for aruco markers,
        the goal point and end point
        return: corners, goal and end points
        """
        cv2_img = imgmsg_to_cv2(image)
        scale_percent = 100 #percent of original image
        width = int(cv2_img.shape[1] * scale_percent / 100)
        height = int(cv2_img.shape[0] * scale_percent / 100)
        dim = (width, height)
        print("shape of original image: ", cv2_img.shape)

        resized_img = cv2.resize(cv2_img, dim, interpolation= cv2.INTER_AREA)

        aruco_corners, goal = get_coordinates(resized_img)

        return aruco_corners, goal
