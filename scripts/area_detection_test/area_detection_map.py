#!/usr/bin/env python
import sys
sys.path.append('../')
import cv2
import cv2.aruco as aruco
import numpy as np
import rogata_library as rgt
import rospy

large_ring      = np.load('large_ring.npy')
large_ring_hole = np.load('large_ring_hole.npy')

small_ring      = np.load('small_ring.npy')
small_ring_hole = np.load('small_ring_hole.npy')

rectangle      = np.load('rectangle.npy')

contour_array = [large_ring,large_ring_hole,small_ring,small_ring_hole]
name          = "rings"
hole_spec     = np.array([1,-1,2,-2])

rings_obj     = rgt.game_object(name,contour_array,hole_spec)
rectangle_obj = rgt.game_object("rectangle",[rectangle],np.array([1]))

if __name__ == "__main__":
    try:
        rospy.init_node('rogata_engine',anonymous=True)
        area_detection_scene = rgt.scene([rings_obj,rectangle_obj])
    except rospy.ROSInterruptException:
        pass
