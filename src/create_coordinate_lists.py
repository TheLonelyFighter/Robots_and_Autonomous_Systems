# Created by Myrthe Tilleman on 14-4-2022.
# Description: Creates a grid map and updates the grid map based on obstacles detected.

import numpy as np
import matplotlib.pyplot as plt
import cv2

from bresenham import bresenham
from recognize_obstacle import detect_aruco_code


def get_coordinates_line(start_point, end_point, limits, vertical=True):
    """ Create a list of coordinates of all points from the start point to end point. """
    x_max = limits[1]
    y_max = limits[0]
    if vertical:
        line = np.arange(np.clip(start_point[1], 0, y_max), np.clip(end_point[1], 0, y_max))
        #line = np.clip((np.arange(start_point[1], end_point[1])), 0, y_max)
        coordinate_list = [(np.clip(start_point[0], 0, x_max), y) for y in line]
    else:  # horizontal line
        line = np.arange(np.clip(start_point[0], 0, x_max), np.clip(end_point[0], 0, x_max))
        #line = np.clip((np.arange(start_point[0], end_point[0])), 0, x_max)
        coordinate_list = [(x, np.clip(start_point[1], 0, y_max)) for x in line]
    return coordinate_list


def get_coordinates(frame):
    """
    Get the coordinates of the aruco codes and creates lists of coordinates of
    the borders of the obstacle, goal area, and the starting position area.
    """
    corners, ids, aruco_size = detect_aruco_code(frame)
    shape = frame.shape[:2]

    obstacle_coordinates = []
    goal_coordinates = []
    starting_coordinates = []
    # loop through detected obstacles
    for i, (top_left, top_right, bottom_right, bottom_left) in enumerate(corners):
        # TODO adapt the coordinates to the location of the drone
        size = aruco_size[i]  # size of the aruco markers in the frame
        buffer = 1 * size

        top_left_buffer = top_left + [-buffer, -buffer]
        top_right_buffer = top_right + [buffer, -buffer]
        bottom_left_buffer = bottom_left + [-buffer, buffer]
        bottom_right_buffer = bottom_right + [buffer, buffer]

        # left_side = get_coordinates_line(top_left_buffer, bottom_left_buffer, shape, vertical=True)
        # right_side = get_coordinates_line(top_right_buffer, bottom_right_buffer, shape, vertical=True)
        # top_side = get_coordinates_line(top_left_buffer, top_right_buffer, shape, vertical=False)
        # bottom_side = get_coordinates_line(bottom_left_buffer, bottom_right_buffer, shape, vertical=False)

        left_side = list(bresenham(top_left_buffer[0], top_left_buffer[1], bottom_left_buffer[0], bottom_left_buffer[1]))
        right_side = list(bresenham(top_right_buffer[0], top_right_buffer[1], bottom_right_buffer[0], bottom_right_buffer[1]))
        top_side = list(bresenham(top_left_buffer[0], top_left_buffer[1], top_right_buffer[0], top_right_buffer[1]))
        bottom_side = list(bresenham(bottom_left_buffer[0], bottom_left_buffer[1], bottom_right_buffer[0], bottom_right_buffer[1]))


        all_coordinates = left_side + right_side + top_side + bottom_side

        if ids[i] == 4:  # goal
            goal_coordinates.extend(all_coordinates)
        elif ids[i] == 0:  # starting point jetbot
            starting_coordinates.extend(all_coordinates)
        else:  # obstacle
            obstacle_coordinates.extend(all_coordinates)

    return obstacle_coordinates, goal_coordinates, starting_coordinates


if __name__ == '__main__':
    image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')

    obstacles, goal, start = get_coordinates(image)