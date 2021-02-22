import  sys
sys.path.append("../")
import cv2
import cv2.aruco as aruco
import numpy as np
import rogata_library as rgt
import time

white_hole    = np.load('white_hole.npy')
right_rock    = np.load('right_rock.npy')
left_rock     = np.load('left_rock.npy')
contour_array = [white_hole,right_rock,left_rock]
name          = "ray_casting_test"
hole_spec     = np.array([1,-1,-1])

obj = rgt.game_object(name,contour_array,hole_spec)

cap = cv2.VideoCapture('ray_casting.mp4')

while(cap.isOpened()):
    marker_id = 6
    ret, image = cap.read()
    #marker detection:
    gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    
    cv2.drawContours(image,white_hole, -1, (0,0,255),3)



    if ids.any() == None:
        print("no marker")
    if marker_id not in ids:
        print("no ids")
    else:
        indice=np.where(ids == marker_id)
        center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4
        test_point=np.array([center[0],center[1]])

        for i in range(36):
            direction = np.array([np.sin(2*np.pi/36*i),np.cos(2*np.pi/36*i)])
            length    = 300
            end_point = obj.line_intersect(test_point,direction,length)
            cv2.line(image,tuple(test_point),(int(end_point[0]),int(end_point[1])),(255,0,0),3)
        cv2.imshow("rac casting test",image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

