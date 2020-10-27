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
    print(corners)
    aruco.drawDetectedMarkers(image,corners)


    #final version only searches for specifc marker ID !
    print(len(contours))
    for contour in contours:
        center = np.sum(corners[0][0],axis=0)/4
        cv2.circle(image,(center[0],center[1]),7,(0,0,255),7)
        #only contour around marker is returned
        if is_inside((center[0],center[1]),contour):
            cv2.drawContours(image, contour, -1, (0,0,255),3)
            return contour
    return None






def calibrate_colors():
    while(1):
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        mid_color = np.array([cv2.getTrackbarPos("H","Test image"),
                              cv2.getTrackbarPos("S","Test image"),
                              cv2.getTrackbarPos("V","Test image")])
        #mid_color = 0.5*(upper_color-lower_color)
        step      = np.array([cv2.getTrackbarPos("H range","Test image"),
                              cv2.getTrackbarPos("S range","Test image"),
                              cv2.getTrackbarPos("V range","Test image")])
        used_img  = image.copy()
        #TODO add floor so that values are never outside of picture range
        detect_area(used_img,mid_color-step,mid_color+step,marker_id)
        cv2.imshow("Test image",used_img)


def nothing(x):
    pass
cv2.namedWindow("Test image")
cv2.createTrackbar("H","Test image",0,179,nothing)
cv2.createTrackbar("H range","Test image",0,50,nothing)
cv2.createTrackbar("S","Test image",0,255,nothing)
cv2.createTrackbar("S range","Test image",0,120,nothing)
cv2.createTrackbar("V","Test image",0,255,nothing)
cv2.createTrackbar("V range","Test image",0,120,nothing)


calibrate_colors()



#print(detect_area(image,lower_color,upper_color,0))
cv2.imwrite("image_sensing_test.png",image)
