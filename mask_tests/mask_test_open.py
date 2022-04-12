import cv2
import numpy as np
import os

for file in os.listdir("../images/open_red_square"):

   #read image
    img = cv2.imread(f'../images/open_red_square/{file}')

    #generate HSV image
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #lower and upper range for masking
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])

    #generate the mask
    mask = cv2.inRange(img_hsv, lower_red, upper_red)

    #extract the masked image from the original image
    img_result = cv2.bitwise_and(img, img, mask=mask)

    #genrate greyscale version for finding contours
    gray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)

    #find contours
    cnts, hierachy = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    #find contour with maximum area
    areas = {}
    max_area = -1
    max_cnt_index = 0
    for i in range(len(cnts)):
        area = cv2.contourArea(cnts[i])
        areas[i] = area
    #ectract only 4 contours with max area
    sort_areas = sorted(areas.items(), key=lambda x: x[1], reverse=True)[:4]

    #get keys for maximum area contours
    keys = []
    for item in sort_areas:
        keys.append(item[0])

    #min x and y, max x and y for rect to draw around the 4 contours
    min_x,min_y = 2000,20000
    max_x, max_y = 0 , 0

    #itterate through each contour and find min and max values
    for key in keys:
        (x,y,w,h) = cv2.boundingRect(cnts[key])
        min_x, max_x = min(x, min_x), max(x+w, max_x)
        min_y, max_y = min(y, min_y), max(y+h, max_y)

    #draw rect around the contours
    if max_x - min_x > 0 and max_y - min_y > 0:
        cv2.rectangle(img_result, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)
    
    cv2.imwrite(f'{file}', img_result)