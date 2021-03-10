#!/usr/bin/env python
import sys
sys.path.append('../')
import cv2
import cv2.aruco as aruco
import numpy as np
import rogata_library as rgt
from rogata_engine.srv import *
import rospy


rospy.wait_for_service('check_inside')
inside_check = rospy.ServiceProxy('check_inside',CheckInside,persistent=True)
 

cap    = cv2.VideoCapture('point_in_area.mp4')
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out    = cv2.VideoWriter('area_detection.avi',fourcc,20.0,(width,height))

while(cap.isOpened()):
    marker_id = 6
    try:
        ret, image = cap.read()
        #marker detection:
        gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    
        try:
            if ids.any() == None:
                print("no marker")
            if marker_id not in ids:
                print("no ids")
        except:
            print('marker not found')
            continue

        else:
            indice=np.where(ids == marker_id)
            center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4
            test_point=np.array([center[0],center[1]])
           
            objects=np.array(['rings','rectangle'])
            for i in range(len(objects)):
                req = CheckInsideRequest(objects[i],test_point[0],test_point[1])
                resp = inside_check(req)
                font_placement = (int(test_point[0]),int(test_point[1]+10-30*i))
                if resp.inside:
                    cv2.putText(image,"Inside "+objects[i],font_placement,cv2.FONT_HERSHEY_SIMPLEX,1,(209, 80, 0, 255), 3) 
                else:
                    cv2.putText(image,"Outside "+objects[i],font_placement,cv2.FONT_HERSHEY_SIMPLEX,1,(209, 80, 0, 255), 3) 

            out.write(image)
            cv2.imshow("rac casting test",image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except:
        break
cap.release()
out.release()
cv2.destroyAllWindows()

