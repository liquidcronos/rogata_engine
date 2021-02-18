import cv2
import cv2.aruco as aruco
import numpy as np
import yaml







'''
This script tests the optical detection static objects, comprised of a colored outline and a aruco marker within.
'''
lower_color            = np.array([0,50,20])      #([71,62,0]) for rgb
upper_color            = np.array([20,255,255])      #([60,255,60]) for rgb
image                  = cv2.imread("ray_casting.jpg")


def is_inside(obj,area):
    if cv2.pointPolygonTest(area,obj, False) >= 0:
        return True
    else:
        return False

#returns contour if any mage area description
def detect_area(image,lower_color,upper_color,marker_id,draw=False):
    # carefull ros will later on need rgb2hsv
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # color detection
    if lower_color[0] <=0:
        second_lower    = lower_color
        second_lower[0] = 179+lower_color[0]
        second_upper    = upper_color
        second_upper[0] = 179

        lower_color[0] = 0
        
        mask1 =cv2.inRange(hsv_img,lower_color,upper_color)
        mask2 =cv2.inRange(hsv_img,second_lower,second_upper)
        mask= mask1 | mask2
    else:
        mask =cv2.inRange(hsv_img,lower_color,upper_color)

    #TODO carefull depending on opencv version the return may be different
    _,contours, hierachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    

    #marker detection:
    gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5
    parameters = aruco.DetectorParameters_create()  
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    
    if draw == True:
        cv2.drawContours(image, contours, -1, 255,3)
    if ids.any() == None:
        return None
    if marker_id not in ids:
        return None
    else:
        indice=np.where(ids == marker_id)
        center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4


    if draw == True:
        cv2.circle(image,(center[0],center[1]),7,(0,0,255),7)
        aruco.drawDetectedMarkers(image,corners)

    #TODO smallest contour should be real contour encompassing whole image
    row, col =hsv_img.shape[:2]
    smallest_contour = np.array([[0,0],[0,row],[col,row],[col,0]])
    #TODO not needet with real contour
    contour_found    = 0
    print( cv2.contourArea(smallest_contour))
    for i in range(len(contours)):
        if is_inside((center[0],center[1]),contours[i]):
            if cv2.contourArea(contours[i]) <= cv2.contourArea(smallest_contour):
                contour_found = 1
                smallest_contour = contours[i]
            if draw == True:
                cv2.drawContours(image, contours[i], -1, (0,0,255),3)

    if contour_found == 1:
        if draw == True:
            print("drew contour")
            cv2.drawContours(image, smallest_contour, -1, (0,255,0),3)
        return smallest_contour

    return None






def calibrate_colors():
    def nothing(x):
        pass
    cv2.namedWindow("Test image")
    cv2.createTrackbar("H","Test image",0,179,nothing)
    cv2.createTrackbar("H range","Test image",0,50,nothing)
    cv2.createTrackbar("S","Test image",0,255,nothing)
    cv2.createTrackbar("S range","Test image",0,120,nothing)
    cv2.createTrackbar("V","Test image",0,255,nothing)
    cv2.createTrackbar("V range","Test image",0,120,nothing)
    cv2.createTrackbar("Marker Id","Test image",0,120,nothing)
    while(1):
        k = cv2.waitKey(1)
        if k == 27:
            break
        if k == ord('s'):
            np.save("contour_save",find_contour)
            break

        mid_color = np.array([cv2.getTrackbarPos("H","Test image"),
                              cv2.getTrackbarPos("S","Test image"),
                              cv2.getTrackbarPos("V","Test image")])
        #mid_color = 0.5*(upper_color-lower_color)
        step      = np.array([cv2.getTrackbarPos("H range","Test image"),
                              cv2.getTrackbarPos("S range","Test image"),
                              cv2.getTrackbarPos("V range","Test image")])

        marker_id =           cv2.getTrackbarPos("Marker Id", "Test image")

        min_value    = np.zeros(3)
        min_value[0] = -179
        max_value    = np.array([179,255,255])
        lower_bound  = np.clip(mid_color-step,min_value,max_value)
        upper_bound  = np.clip(mid_color+step,min_value,max_value)

        used_img     = image.copy()
        find_contour = detect_area(used_img,lower_bound,upper_bound,marker_id,True)
        cv2.imshow("Test image",used_img)




calibrate_colors()



#print(detect_area(image,lower_color,upper_color,0))
cv2.imwrite("image_sensing_test.png",image)
