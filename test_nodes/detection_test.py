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
marker_id              = 0


def is_inside(obj,area):
    if cv2.pointPolygonTest(area,obj, False) >= 0:
        return True
    else:
        return False

#returns contour if any mage area description
def detect_area(image,lower_color,upper_color,marker_id):
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
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    aruco.drawDetectedMarkers(image,corners)


    #final version only searches for specifc marker ID !
    for contour in contours:
        center = np.sum(corners[0][0],axis=0)/4
        cv2.circle(image,(center[0],center[1]),7,(0,0,255),7)
        #only contour around marker is returned
        if is_inside((center[0],center[1]),contour):
            cv2.drawContours(image, contour, -1, (0,0,255),3)
            return contour
    return None



#print(detect_area(image,lower_color,upper_color,0))
cv2.imwrite("image_sensing_test.png",image)
