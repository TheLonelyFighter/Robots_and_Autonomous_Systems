#!/usr/bin/env python

'''
Welcome to the ArUco Marker Detector!
  
This program:
  - Detects ArUco markers using OpenCV and Python
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import math
 
# Project: ArUco Marker Detector
# Date created: 12/18/2021
# Python version: 3.8
# Reference: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
 
desired_aruco_dictionary = "DICT_4X4_50"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }

def calculate_angle(reference_point, inclined_point):
    """
    Calculate the angle of the square between a reference line and the side.
    Also calculate the size of one side.
    """
    side = inclined_point - reference_point

    ref_point = [inclined_point[0], reference_point[1]]  # point of reference
    reference_line = ref_point - reference_point

    # define x and y values of the two vectors
    x1, y1 = side
    x2, y2 = reference_line

    # calculate length of the side
    side_length = math.sqrt(x2 ** 2 + y1 ** 2)

    # calculate the angle
    angle_rad = math.acos(x2 / side_length)
    return angle_rad, side_length


def calculate_centre_coordinates(angle, radius, centre_qr, marker_id):
    """
    Calculates the center coordinates of a circle based on the centre coordinates and the angle of the
    qr code, as well as the marker_id and the circle radius.
    """
    center_x, center_y = centre_qr
    if marker_id == 1:  # bottom marker
        centre_circle_x = -math.sin(angle) * radius + center_x
        centre_circle_y = -math.cos(angle) * radius + center_y
    elif marker_id == 2:  # top marker
        centre_circle_x = math.sin(angle) * radius + center_x
        centre_circle_y = math.cos(angle) * radius + center_y
    elif marker_id == 3:  # right
        centre_circle_x = -math.cos(angle) * radius + center_x
        centre_circle_y = math.sin(angle) * radius + center_y
    elif marker_id == 4:  # left
        centre_circle_x = math.cos(angle) * radius + center_x
        centre_circle_y = -math.sin(angle) * radius + center_y
    else:
        print('Another marker has been found with ID %i' % marker_id)
        centre_circle_x = np.nan
        centre_circle_y = np.nan

    return [centre_circle_x, centre_circle_y]


def main():
    """
    Main method of the program.
    """
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))
        sys.exit(0)
        
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(
        desired_aruco_dictionary))
    this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
        
    frame = cv2.imread('../images/new_markers/qr_codes/frame0025.jpg')

    # Detect ArUco markers in the video frame
    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, this_aruco_dictionary, parameters=this_aruco_parameters)

    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
    # Flatten the ArUco IDs list
        ids = ids.flatten()

    # Loop over the detected ArUco corners
    centre_circle = []
    avg_radius = []

    for (marker_corner, marker_id) in zip(corners, ids):
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert the (x,y) coordinate pairs to integers
        top_right = np.array((int(top_right[0]), int(top_right[1])))
        bottom_right = np.array((int(bottom_right[0]), int(bottom_right[1])))
        bottom_left = np.array((int(bottom_left[0]), int(bottom_left[1])))
        top_left = np.array((int(top_left[0]), int(top_left[1])))

        # Draw the bounding box of the ArUco detection
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

        # Calculate and draw the center of the ArUco marker
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

        # Draw the ArUco marker ID on the video frame
        # The ID is always located at the top_left of the ArUco marker
        cv2.putText(frame, str(marker_id),
                    (top_left[0], top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        # calculate the angle and the radius
        angle, side_length = calculate_angle(top_left, bottom_left)
        radius_gap = 4 * side_length  # radius of the gap to fly through
        radius_circle = radius_gap + 0.5 * side_length  # radius from centre to centre of the codes
        avg_radius.append(radius_gap)

        circle_coordinates = calculate_centre_coordinates(angle, radius_circle, (center_x, center_y), marker_id)
        centre_circle.append(circle_coordinates)

    # calculate the mean radius and centre of the circles
    centre_circle = np.array(centre_circle)
    centre_circle = np.round(np.nanmean(centre_circle, axis=0)).astype(int)
    avg_radius = int(np.round(np.mean(avg_radius)))

    # draw the circle and the middle point
    cv2.circle(frame, centre_circle, avg_radius, color=(255, 0, 0), thickness=2)
    cv2.circle(frame, centre_circle, 4, color=(0, 0, 255), thickness=-1)

    cv2.imwrite('../images/detected/detected3.jpg', frame)
    # Display the resulting frame
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print(__doc__)
    main()
