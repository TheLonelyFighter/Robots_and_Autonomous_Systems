# Created by Myrthe Tilleman on 14-4-2022.
# Description: Recognizes obstacles in a frame based on AruCo markers.
# Gives the center coordinates and marker of the code.

import cv2
import numpy as np

from recognize_qr_gate import draw_aruco_markers, convert_to_integer_pair, calculate_angle


def detect_aruco_code(frame):
    """ Detects the aruco code in a frame and returns the coordinates. """
    ARUCO_DICT = cv2.aruco.DICT_4X4_250
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

        frame, center_x, center_y = draw_aruco_markers(frame, marker_id, top_left, top_right, bottom_left, bottom_right)
        # center_coordinates.append((center_x, center_y))q
        corners = [top_left, top_right, bottom_right, bottom_left]
        all_corners.append(corners)

        #cv2.aruco.drawDetectedMarkers(frame, rejected)

        _, size = calculate_angle(top_left, bottom_left)
        # cv2.putText(frame, str(np.round(size).astype(int)), (bottom_left[0], bottom_left[1] - 15),
        #             cv2.FONT_HERSHEY_SIMPLEX,
        #             0.5, (0, 255, 0), 2)
        aruco_size.append(np.round(size).astype(int))

    # cv2.imshow("Image", frame)
    # cv2.waitKey(0)
    return all_corners, ids, aruco_size


if __name__ == '__main__':
    image = cv2.imread('../images/obstacle_course/test1.jpg')

    scale_percent = 100 #percent of original image
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    print("shape of original image: ", image.shape)

    resized_img = cv2.resize(image, dim, interpolation= cv2.INTER_AREA)
    aruco_corners, marker_ids, aruco_size = detect_aruco_code(resized_img)
    print(marker_ids)
    print(len(marker_ids))
