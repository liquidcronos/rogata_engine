import  sys
sys.path.append("../")
import cv2
import cv2.aruco as aruco
import numpy as np
import rogata_library as rgt
import time

visibility_area = np.load('visibility_box.npy')
contour_array   = [visibility_area]
name            = "visibility_test"
hole_spec       = np.array([1])

obj = rgt.game_object(name,contour_array,hole_spec)

cap = cv2.VideoCapture('visibility_calculation.mp4')


def aruco_center(marker_id,gray_image):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 4x4
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters, ids=marker_id)

    indice=np.where(ids == marker_id)
    center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4
    return center

def visibility(guard,thief,eps=2):
    distance  = np.linalg.norm(thief-guard)
    direction = (thief-guard)/distance

    seeing_distance = obj.line_intersect(guard,direction,800)
    cv2.line(image,tuple(guard),tuple(seeing_distance),(255,0,0),3)
    if np.linalg.norm(seeing_distance-guard) >= distance:
        return 1
    else:
        return 0


while(cap.isOpened()):
    ret, image = cap.read()
    gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    
    guard      = aruco_center(0,gray_image)
    thief      = aruco_center(1,gray_image)
    
    
    font_placement = (int(thief[0]),int(thief[1])-10)
    if visibility(guard,thief):
        cv2.putText(image,"Visible", font_placement, cv2.FONT_HERSHEY_SIMPLEX,1,(209,80,0,255),3)
    else:
        cv2.putText(image,"Hidden", font_placement, cv2.FONT_HERSHEY_SIMPLEX,1,(209,80,0,255),3)



    cv2.imshow("rac casting test",image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

