#!/usr/bin/env python
  
'''
Welcome to the ArUco Marker Detector!
  
This program:
  - Detects ArUco markers using OpenCV and Python
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
 
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
        
    frame = cv2.imread('images/new_markers/qr_codes/frame0025.jpg')

    # Detect ArUco markers in the video frame
    # for _, dictionary in enumerate(ARUCO_DICT):
    #     print(dictionary)
    #     this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[dictionary])
    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, this_aruco_dictionary, parameters=this_aruco_parameters)
        # if len(corners) == 0:
        #     pass
        # else:
        #     print(ids)
        #     break

    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
    # Flatten the ArUco IDs list
        ids = ids.flatten()

    # Loop over the detected ArUco corners
    for (marker_corner, marker_id) in zip(corners, ids):
    
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
        
        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))

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

        # draw a circle
        image_size = frame.shape
        size_x = np.abs(top_right[0] - top_left[0])
        size_y = np.abs(top_left[1] - bottom_left[1])
        radius = 5 * size_x

        centre_circle_x = []
        centre_circle_y = []

        if marker_id == 1:  # bottom marker
            centre_circle_x.append(center_x)
            centre_circle_y.append(center_y + radius)
        elif marker_id == 2:  # top marker
            centre_circle_x.append(center_x)
            centre_circle_y.append(center_y - radius)
        elif marker_id == 3:  # right
            centre_circle_x.append(center_x + radius)
            centre_circle_y.append(center_y)
        elif marker_id == 4:  # left
            centre_circle_x.append(center_x - radius)
            centre_circle_y.append(center_y)

    centre_circle_x = int(np.round(np.mean(centre_circle_x)))
    centre_circle_y = int(np.round(np.mean(centre_circle_y)))
    cv2.circle(frame, (centre_circle_x, centre_circle_y), radius, color=(255, 0, 0), thickness=2)

    cv2.imwrite('images/detected.jpg', frame)
    # Display the resulting frame
    cv2.imshow('frame', frame)


if __name__ == '__main__':
    print(__doc__)
    main()
