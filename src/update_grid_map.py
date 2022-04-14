# Created by Myrthe Tilleman on 14-4-2022.
# Description: Creates a grid map and updates the grid map based on obstacles detected.

import numpy as np
import matplotlib.pyplot as plt
import cv2

from recognize_obstacle import detect_aruco_code


def create_grid_map(size):
    """ Creates a grid map. """
    map = np.zeros(size)
    return map


def update_grid_map(map, frame):
    """ Get the coordinates of the aruco codes and updates the grid map. """
    center_coordinates, ids, frame = detect_aruco_code(frame)
    size = 5  # size of the obstacles and goal in grid map

    # loop through detected obstacles
    for i, center in enumerate(center_coordinates):
        # TODO adapt the coordinates to the location of the drone
        x, y = center
        if ids[i] == 1:  # goal
            map[x - size: x + size, y - size: y + size] = 2
        else:  # obstacle
            map[x - size: x + size, y - size: y + size] = -1

    return map


if __name__ == '__main__':
    image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')

    grid = create_grid_map((1000, 1000))  # initial grid
    updated_grid = update_grid_map(grid, image)

    image2 = cv2.imread('../images/new_markers/qr_codes/frame0028.jpg')
    updated_grid = update_grid_map(updated_grid, image2)

    plt.matshow(updated_grid)
    plt.savefig('updated_grid.png')
