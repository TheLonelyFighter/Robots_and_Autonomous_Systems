# Created by Myrthe Tilleman on 14-4-2022.
# Description: Recognizes obstacles in a frame based on AruCo markers.
# Gives the center coordinates and marker of the code.

import cv2
import numpy as np

from recognize_qr_gate import draw_aruco_markers, convert_to_integer_pair, calculate_angle


def detect_aruco_code(frame):
    """ Detects the aruco code in a frame and returns the coordinates. """
    ARUCO_DICT = cv2.aruco.DICT_4X4_50
    aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT)

    # Detect ArUco markers in the video frame
    aruco_parameters = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, aruco_dictionary, parameters=aruco_parameters)

    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
    # Flatten the ArUco IDs list
        ids = ids.flatten()
    else:  # if no markers are detected
        return None, None, None

    center_coordinates = []
    all_corners = []
    aruco_size = []
    # loop over detected markers
    for (marker_corner, marker_id) in zip(corners, ids):
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert the (x,y) coordinate pairs to integers
        top_right = convert_to_integer_pair(top_right)
        bottom_right = convert_to_integer_pair(bottom_right)
        bottom_left = convert_to_integer_pair(bottom_left)
        top_left = convert_to_integer_pair(top_left)

        # frame, center_x, center_y = draw_aruco_markers(frame, marker_id, top_left, top_right, bottom_left, bottom_right)
        # center_coordinates.append((center_x, center_y))
        corners = [top_left, top_right, bottom_right, bottom_left]
        all_corners.append(corners)

        _, size = calculate_angle(top_left, bottom_left)
        aruco_size.append(np.round(size).astype(int))
    return all_corners, ids, aruco_size


if __name__ == '__main__':
    image = cv2.imread('../images/new_markers/qr_codes/frame0032.jpg')
    aruco_corners, marker_ids, aruco_size = detect_aruco_code(image)
    print(aruco_corners, marker_ids)
