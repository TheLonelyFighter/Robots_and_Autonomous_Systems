import cv2
import numpy as np
from matplotlib import pyplot as plt
import os

for file in os.listdir("../images/red_square"):
    #print(file)
  
    # reading image
    img = cv2.imread(f'../images/red_square/{file}')
    
    #img=cv2.imread('img.bmp')
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50]) #example value
    upper_red = np.array([10,255,255]) #example value
    mask = cv2.inRange(img_hsv, lower_red, upper_red)
    img_result = cv2.bitwise_and(img, img, mask=mask)

    gray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
    cnts, hierachy = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    #print(cnts)

    print("Number of Contours found = " + str(len(cnts)))

    max_area = -1
    max_cnt_index = 0
    for i in range(len(cnts)):
        area = cv2.contourArea(cnts[i])
        if area>max_area:
            max_cnt_index = i
            max_area = area

    #for cont in cnts:
    #    cv2.drawContours(img_result, cnts, -1, (0,255,0), 3)

    cv2.drawContours(img_result, cnts, max_cnt_index, (0,255,0), 3)

    #rint(cnts[max_cnt_index].tolist())

    min_x = 1000
    min_y =1000
    max_x = 0
    max_y = 0

    for item in cnts[max_cnt_index].tolist():
        #print(item[0][0])
        if item[0][0] < min_x:
            min_x = item[0][0]
        if item[0][0] > max_x:
            max_x = item[0][0]

        if item[0][1] < min_y:
            min_y = item[0][1]
        if item[0][1] > max_y:
            max_y = item[0][1]

    x,y,w,h = cv2.boundingRect(cnts[max_cnt_index])
    cv2.rectangle(img_result,(x,y),(x+w,y+h),(0,255,0),1)
    
    cv2.imwrite(f'{file}', img_result)
    
    # displaying the image after drawing contours
    #cv2.imshow('shapes', img_result)
    
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()