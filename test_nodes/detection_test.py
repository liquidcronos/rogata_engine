import cv2
import cv2.aruco as aruco
import numpy as np

from random import randrange



'''
This script tests the optical detection static objects, comprised of a colored outline and a aruco marker within.
'''
lower_color            = np.array([0,50,20])      #([71,62,0]) for rgb
upper_color            = np.array([20,255,255])      #([60,255,60]) for rgb
image                  = cv2.imread("test_image_2.jpg")

# carefull ros will later on need rgb2hsv
hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# color detection
mask =cv2.inRange(hsv_img,lower_color,upper_color)
contours, hierachy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#visualize contours:
cv2.drawContours(image, contours, -1, 255,3)


#marker detection:
gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5
parameters = aruco.DetectorParameters_create()  
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
aruco.drawDetectedMarkers(image,corners)

for contour in contours:
    for marker in corners:
        center = np.sum(marker[0],axis=0)/4
        cv2.circle(image,(center[0],center[1]),7,(0,0,255),7)
        # check if aruco marker is inside contour 
        print(cv2.pointPolygonTest(contour,(center[0],center[1]), False))
        if cv2.pointPolygonTest(contour,(center[0],center[1]), False) >= 0:
            cv2.drawContours(image, contour, -1, (0,0,255),3)

biggest_contour=max(contours, key=cv2.contourArea)
#cv2.drawContours(image,biggest_contour,-1,(0,0,255),3)



cv2.imwrite("image_sensing_test.png",image)
