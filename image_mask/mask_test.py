import cv2
import numpy as np
import os

for file in os.listdir("../images/red_square"):

    #read image
    img = cv2.imread(f'../images/red_square/{file}')
    
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
    max_area = -1
    max_cnt_index = 0
    for i in range(len(cnts)):
        area = cv2.contourArea(cnts[i])
        if area>max_area:
            max_cnt_index = i
            max_area = area

    #Draw contour around the one with maximum area. The index of the contour with maxiumum area is assigned to max_cnt_index in previous bloack of code
    cv2.drawContours(img_result, cnts, max_cnt_index, (0,255,0), 3)

    #draw rectangle around the contour
    x,y,w,h = cv2.boundingRect(cnts[max_cnt_index])
    cv2.rectangle(img_result,(x,y),(x+w,y+h),(0,255,0),1)
    
    cv2.imwrite(f'{file}', img_result)