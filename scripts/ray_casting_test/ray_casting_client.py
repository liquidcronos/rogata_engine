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

cap    = cv2.VideoCapture('ray_casting.mp4')
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out    = cv2.VideoWriter('ray_casting.avi',fourcc,20.0,(width,height))


while(cap.isOpened()):
    try:
        ret, image = cap.read()
        gray       = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        rgt.track_dynamic_objects(gray,['robot'])
        

        #test_point=np.array([center[0],center[1]])
        test_point=np.array([0,0])

        length    = 300

        for i in range(36):
            objects     = np.array(['white hole','right rock','left rock'])
            color_array = np.array([[255,0,0],[0,255,0],[0,0,255]])
            for k in range(len(objects)):
                line      = Pose2D(test_point[0],test_point[1],(10*i+4*k)/360.0*2*np.pi)
                name=String()
                name.data = objects[k]
                req       = RequestInterRequest(str(objects[k]),line,length)
                response  = inters(req)
                end_point = (int(response.x),int(response.y))

                cv2.line(image,tuple(test_point),end_point,(color_array[k][0],color_array[k][1],color_array[k][2]),3)

        out.write(image)
        cv2.imshow("rac casting test",image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        break
cap.release()
out.release()
cv2.destroyAllWindows()

