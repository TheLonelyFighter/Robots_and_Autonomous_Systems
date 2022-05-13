import cv2
import numpy as np 
import math
import os
import random

#formula for calculating the real length using image
# f = (PxD)/l
# where P - Pixel Length capture in Image
#       f - Focal length constant here it is 1920
#       D - Distance of image from the camera (Supposed to be height captured by motion capture)
#       l - real length of the object

desired_aruco_dictionary = "DICT_4X4_250"

corner_pose = [2450,1964]
#corner_id should be string
corner_id = '103'
 
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


#earlier f was 1920

f=940
d=1200
f_x = 160

dictionary_aruco = {}

for file in os.listdir("./images"):

    if ".JPG" in file:

        print(file)

        #read image
        img = cv2.imread(f'./images/{file}')
        #print(img.shape)

        da  = file.split(".")
        da = da[0].split("_")
        da = da[0]
        d=int(da)
        #print(d)

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)

        #print(f'Ids are {ids}')

        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                p1 = math.sqrt(((topRight[0] - topLeft[0])**2) + ((topRight[1] -topLeft[1])**2))
                p2 = math.sqrt(((bottomRight[0] - bottomLeft[0])**2) + ((bottomRight[1] -bottomLeft[1])**2))
                p = max(p1,p2)

                real_l = (p*d)/f

                # draw the bounding box of the ArUCo detection
                cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                real_x = (cX*d)/f
                real_y = (cY*d)/f

                topLeft_x = (topLeft[0]*d)/f
                topLeft_y = (topLeft[1]*d)/f

                bottomLeft_x = (bottomLeft[0]*d)/f
                bottomLeft_y = (bottomLeft[1]*d)/f
                
                topRight_x = (topRight[0]*d)/f
                topRight_y = (topRight[1]*d)/f

                bottomRight_x = (bottomRight[0]*d)/f
                bottomRight_y = (bottomRight[1]*d)/f

                

                #print((topLeft_x, topLeft_y), (bottomLeft_x, bottomLeft_y), (topRight_x, topRight_y))
                #print(real_x, real_y)

                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                #print("[INFO] ArUco marker ID: {} with lenght {}".format(markerID, l))

                print("[INFO] ArUco marker ID: {} with lenght {} at x {} and y {}".format(markerID, real_l, real_x, real_y))


                if not f'{markerID}' in dictionary_aruco:
                    dictionary_aruco[f'{markerID}'] = {}
                    dictionary_aruco[f'{markerID}']["cX"] = real_x
                    dictionary_aruco[f'{markerID}']['cY'] = real_y
                    dictionary_aruco[f'{markerID}']['corners'] = [[topLeft_x,topLeft_y], [bottomLeft_x, bottomLeft_y], [topRight_x, topRight_y], [bottomRight_x, bottomRight_y]]
                else:
                    temp_id = f'{markerID}-{random.randint(0,100)}'
                    dictionary_aruco[temp_id] = {}
                    dictionary_aruco[temp_id]["cX"] = real_x
                    dictionary_aruco[temp_id]['cY'] = real_y
                    dictionary_aruco[temp_id]['corners'] = [[topLeft_x,topLeft_y], [bottomLeft_x, bottomLeft_y], [topRight_x, topRight_y], [bottomRight_x, bottomRight_y]]
    

        # show the output image	
        cv2.imshow("Image", img)
        cv2.waitKey(0)

#print(dictionary_aruco)

#print(dictionary_aruco['103'])

pic_start_x = corner_pose[0] + dictionary_aruco[corner_id]['cX']
pic_start_y = corner_pose[1] - dictionary_aruco[corner_id]['cY']

for key in dictionary_aruco:
    dictionary_aruco[key]['cX'] = pic_start_x - dictionary_aruco[key]['cX']
    dictionary_aruco[key]['cY'] = pic_start_y + dictionary_aruco[key]['cY']

    if key == '102' or key == '103' or key ==  '101' or  key == '104':
        print(key,dictionary_aruco[key])