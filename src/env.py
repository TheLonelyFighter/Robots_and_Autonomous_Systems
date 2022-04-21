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
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')
        aruco_corners, goal_point, starting_point = get_coordinates(image)
        print(f'length of aruco_corners = {len(aruco_corners)}.')
        #print(aruco_corners)

        return aruco_corners
