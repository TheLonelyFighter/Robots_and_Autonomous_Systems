"""
Env 2D
@author: huiming zhou
"""

import cv2
from create_coordinate_lists import get_coordinates

class Env:
    def __init__(self):
        self.x_range = 960  # size of background
        self.y_range = 720
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

        # self.motions  =  [  ( 1, 0 ), # go up
        #                     ( 0, -1), # go left
        #                     ( 1, 0 ), # go down
        #                     ( 0, 1 )] # go right
        self.obs, self.goal_points, self.start_points = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        image = cv2.imread('../images/obstacle_course/test1.jpg')

        scale_percent = 50 #percent of original image
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)

        resized_img = cv2.resize(image, dim, interpolation= cv2.INTER_AREA)

        # cv2.imshow('resized', resized_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        aruco_corners, goal_points, starting_points = get_coordinates(resized_img)
        #print(f'length of aruco_corners = {len(aruco_corners)}.')
        #print(aruco_corners)

        return aruco_corners, goal_points, starting_points
