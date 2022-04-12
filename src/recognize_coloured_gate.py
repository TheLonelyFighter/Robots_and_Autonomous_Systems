# Created by Myrthe Tilleman on 12-4-2022.
# Description:  Detects a coloured gate and calculates it center.

import cv2
import numpy as np


def calculate_center_full(image, contours, max_index):
    """
    Calculates the center of a detected fully coloured gate.
    Draw contour around the one with maximum area.
    The index of the contour with maximum area is assigned with max_index.
    """
    cv2.drawContours(image, contours, max_index, (0, 255, 0), 3)

    # draw rotated rectangle using minAreaRect
    rect = cv2.minAreaRect(cnts[max_index])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img_result, [box], 0, (0, 0, 255), 3)

    # find the center of the rectangle and draw it on the image
    center = (int(rect[0][0]), int(rect[0][1]))
    cv2.circle(image, center, 10, (0, 0, 255), 3)

    # calculate size
    width = rect[1][0]
    size = width / 2
    return center, size


def calculate_center_open(image, contours):
    """
    Calculates the center of a detected open coloured gate.
    Draw contour around the 4 biggest areas.
    """
    # find contour with maximum area
    areas = {}
    max_area = -1
    max_cnt_index = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        areas[i] = area

    # extract only 4 contours with max area
    sort_areas = sorted(areas.items(), key=lambda x: x[1], reverse=True)[:4]

    # get keys for maximum area contours
    keys = []
    for item in sort_areas:
        keys.append(item[0])

    # min x and y, max x and y for rect to draw around the 4 contours
    min_x, min_y = 2000, 20000
    max_x, max_y = 0, 0

    # iterate through each contour and find min and max values
    for key in keys:
        (x, y, w, h) = cv2.boundingRect(contours[key])
        min_x, max_x = min(x, min_x), max(x + w, max_x)
        min_y, max_y = min(y, min_y), max(y + h, max_y)

    # draw rect around the contours
    if max_x - min_x > 0 and max_y - min_y > 0:
        rect = cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)
        center = ((min_x + max_x) / 2, (min_y + max_y) / 2)
        size = (max_x - min_x) / 2

    return center, size


if __name__ == '__main__':
    pass
