#!/usr/bin/env python
import  sys
sys.path.append("../")
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import rogata_library as rgt
from rogata_engine.srv import *
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_srvs.srv import *


# package needed service as function 
# TODO do this automatically somehow?

rospy.wait_for_service('intersect_line')
inters = rospy.ServiceProxy('intersect_line',RequestInter,persistent=True)

cap = cv2.VideoCapture('ray_casting.mp4')



while(cap.isOpened()):
    marker_id = 6
    ret, image = cap.read()
    #marker detection:
    gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    



    if ids.any() == None:
        print("no marker")
    if marker_id not in ids:
        print("no ids")
    else:
        indice=np.where(ids == marker_id)
        center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4
        test_point=np.array([center[0],center[1]])

        length    = 300

        for i in range(36):
            objects     = np.array(['white hole','right rock','left rock'])
            color_array = np.array([[255,0,0],[0,255,0],[0,0,255]])
            for k in range(len(objects)):
                line      = Pose2D(test_point[0],test_point[1],i)
                name=String()
                name.data = objects[k]
                req       = RequestInterRequest(str(objects[k]),line,length)
                response  = inters(req)
                end_point = (int(response.x),int(response.y))

                cv2.line(image,tuple(test_point),end_point,(color_array[k][0],color_array[k][1],color_array[k][2]),6-2*k)
        cv2.imshow("rac casting test",image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

