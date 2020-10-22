#!/usr/bin/env python

## Simple publisher that publish to 'listener_cam' topic


import rospy
import cv2 as cv
import estimating_tools as et
from std_msgs.msg import Float64 as float_

def talker_cam():
	#init Calibration
	cal = et.Calibration()
	#get cam param
	ret, mtx, dist, rvecs, tvecs = cal.getCamParam()
	mrk = et.aruco_marker.Marker()
	pub = rospy.Publisher('talker_cam', float_, queue_size=10)
	rospy.init_node('talker_cam_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		#hello_str = "hello world %s" % rospy.get_time()
		corners , ids , rejectedImgPoints , frame, detected = mrk.detectMarkerLive()
		if detected == True:
			rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers( corners, 0.07, mtx, dist)
			msg0 = float(tvecs[0][0][0])
			msg1 = float(tvecs[0][0][1])
			msg2 = float(tvecs[0][0][2])
			msg = msg0
			rospy.loginfo(msg)
			pub.publish(msg)	
			rate.sleep()

if __name__ == '__main__':
    try:
        talker_cam()
    except rospy.ROSInterruptException:
        pass
