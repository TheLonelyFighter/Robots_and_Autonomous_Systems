"""
Env 2D
@author: huiming zhou
"""

import cv2
from create_coordinate_lists import get_coordinates

class Env:
    def __init__(self):
        self.image = cv2.imread('../images/obstacle_course/test1.jpg')
        self.x_range = self.image.shape[1]  # size of background
        self.y_range = self.image.shape[0]
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

        self.obs, self.goal_points, self.start_points = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        

        scale_percent = 50 #percent of original image
        width = int(self.image.shape[1] * scale_percent / 100)
        height = int(self.image.shape[0] * scale_percent / 100)
        self.x_range = width
        self.y_range = height
        dim = (width, height)
        print(dim)

        resized_img = cv2.resize(self.image, dim, interpolation= cv2.INTER_AREA)

        # cv2.imshow('resized', resized_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        aruco_corners, goal_points, starting_points = get_coordinates(resized_img)
        #print(f'length of aruco_corners = {len(aruco_corners)}.')
        #print(aruco_corners)

        return aruco_corners, goal_points, starting_points
