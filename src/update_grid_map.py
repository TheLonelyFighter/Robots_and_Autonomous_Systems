# Created by Myrthe Tilleman on 14-4-2022.
# Description: Creates a grid map and updates the grid map based on obstacles detected.

import numpy as np
import matplotlib.pyplot as plt
import cv2

from recognize_obstacle import detect_aruco_code


def create_grid_map(frame):
    """ Creates a grid map. """
    size = frame.shape[:2]
    map = np.zeros(size)
    return map


def update_grid_map(map, frame):
    """ Get the coordinates of the aruco codes and updates the grid map. """
    corners, ids, aruco_size = detect_aruco_code(frame)

    # loop through detected obstacles
    for i, (top_left, top_right, bottom_right, bottom_left) in enumerate(corners):
        # TODO adapt the coordinates to the location of the drone
        # x, y = center
        size = aruco_size[i]  # size of the aruco markers in the frame
        buffer = 2 * size

        top_left_buffer = top_left + [-buffer, -buffer]
        print(top_left_buffer)
        top_right_buffer = top_right + [buffer, -buffer]
        bottom_left_buffer = bottom_left + [-buffer, buffer]
        bottom_right_buffer = bottom_right + [buffer, buffer]
        print(bottom_left_buffer)

        left_side_y = np.arange(top_left_buffer[1], bottom_left_buffer[1])
        left_side = [[top_left_buffer[0], y] for y in left_side_y]
        print(left_side)
        # TODO add x-coordinates, to create lists of coordinates of the borders of the obstacles
        # TODO create separate lists for the obstacles, goal (all coordinates, not just borders), and starting point
        obstacle_coordinates = left_side

        # if ids[i] == 1:  # goal
        #     map[y - size: y + size, x - size: x + size] = 2  # [rows, columns]
        # elif ids[i] == 4:  # starting point jetbot
        #     map[y - size: y + size, x - size: x + size] = 5  # [rows, columns]
        # else:  # obstacle
        #     map[y - size: y + size, x - size: x + size] = -1

    return map, obstacle_coordinates


if __name__ == '__main__':
    image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')

    grid = create_grid_map(image)  # initial grid, size of the image
    updated_grid = update_grid_map(grid, image)

    # image2 = cv2.imread('../images/new_markers/qr_codes/frame0028.jpg')
    # updated_grid = update_grid_map(updated_grid, image2)

    plt.matshow(updated_grid)
    plt.savefig('updated_grid.png')
