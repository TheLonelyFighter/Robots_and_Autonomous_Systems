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
        coordinate_list = [(np.clip(start_point[0], 0, x_max), y) for y in line]
    else:  # horizontal line
        line = np.arange(np.clip(start_point[0], 0, x_max), np.clip(end_point[0], 0, x_max))
        coordinate_list = [(x, np.clip(start_point[1], 0, y_max)) for x in line]
    return coordinate_list


def get_coordinates(frame):
    """
    Get the coordinates of the aruco codes and creates lists of coordinates of
    the borders of the obstacle, goal area, and the starting position area.
    """
    corners, ids, aruco_size = detect_aruco_code(frame)
    shape = frame.shape[:2]
    print(f'shape: {shape}.')

    obstacle_coordinates = []
    corner_marker = {}
    # loop through detected obstacles
    for i, (top_left, top_right, bottom_right, bottom_left) in enumerate(corners):
        buffer = 50
        all_coordinates = []

        top_left_buffer = top_left + [-buffer, -buffer]
        top_right_buffer = top_right + [buffer, -buffer]
        bottom_left_buffer = bottom_left + [-buffer, buffer]
        bottom_right_buffer = bottom_right + [buffer, buffer]

        center_rectangle = (int((top_left[0]+bottom_right[0])/2), int((top_left[1]+bottom_right[1])/2))

        left_side = list(bresenham(top_left_buffer[0], top_left_buffer[1], bottom_left_buffer[0], bottom_left_buffer[1]))
        right_side = list(bresenham(top_right_buffer[0], top_right_buffer[1], bottom_right_buffer[0], bottom_right_buffer[1]))
        top_side = list(bresenham(top_left_buffer[0], top_left_buffer[1], top_right_buffer[0], top_right_buffer[1]))
        bottom_side = list(bresenham(bottom_left_buffer[0], bottom_left_buffer[1], bottom_right_buffer[0], bottom_right_buffer[1]))
        

        all_coordinates = left_side + right_side + top_side + bottom_side

        if ids[i] == 0:  # goal
            goal_coordinates = center_rectangle
        elif ids[i] in [101, 102, 103, 104]:  # corner marker
            corner_marker[ids[i]] = center_rectangle
        elif ids[i] == 3:  # obstacle
            obstacle_coordinates.extend(all_coordinates)
        else:
            print("Rejected markers found: ", i)

    if(len(obstacle_coordinates) == 0):
        obstacle_opti = []
    else:
        obstacle_opti = np.unique([map_range(obstacle, corner_marker) for obstacle in obstacle_coordinates], axis=0).tolist()
    goal_opti = map_range(goal_coordinates, corner_marker)

    return obstacle_opti, goal_opti


def map_range(input_coordinates, corner_marker):
    """
    Converts pixels to optitrack coordinates based on coordinates of markers in image
    with calibrated optitrack coordinates.
    """

    # optitrack coordinates of markers
    marker_101 = [85, 189] #bottom_left
    marker_102 = [224, 288] #top_right
    # marker_103 = [215, 197] #bottom_right
    # marker_104 = [102, 307] #top_left

    x, y = input_coordinates

    in_min_x, in_max_x = corner_marker[101][0], corner_marker[102][0]
    out_min_x, out_max_x = marker_101[0], marker_102[0]
    x_new = (x - in_min_x) * (out_max_x - out_min_x) // (in_max_x - in_min_x) + out_min_x

    in_min_y, in_max_y = corner_marker[101][1], corner_marker[102][1]
    out_min_y, out_max_y = marker_101[1], marker_102[1]
    y_new = (y - in_min_y) * (out_max_y - out_min_y) // (in_max_y - in_min_y) + out_min_y
    return [x_new, y_new]


if __name__ == '__main__':
    image = cv2.imread('../images/obstacle_course/test1.jpg')

    obstacles, goal, start = get_coordinates(image)