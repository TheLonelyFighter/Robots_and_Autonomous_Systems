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
    center_coordinates, ids, aruco_size = detect_aruco_code(frame)

    # loop through detected obstacles
    for i, center in enumerate(center_coordinates):
        # TODO adapt the coordinates to the location of the drone
        x, y = center
        size = aruco_size[i]  # size of the aruco markers in the frame
        if ids[i] == 1:  # goal
            map[y - size: y + size, x - size: x + size] = 2  # [rows, columns]
        elif ids[i] == 4:  # starting point jetbot
            map[y - size: y + size, x - size: x + size] = 5  # [rows, columns]
        else:  # obstacle
            map[y - size: y + size, x - size: x + size] = -1

    return map


if __name__ == '__main__':
    image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')

    grid = create_grid_map(image)  # initial grid, size of the image
    updated_grid = update_grid_map(grid, image)

    # image2 = cv2.imread('../images/new_markers/qr_codes/frame0028.jpg')
    # updated_grid = update_grid_map(updated_grid, image2)

    plt.matshow(updated_grid)
    plt.savefig('updated_grid.png')
