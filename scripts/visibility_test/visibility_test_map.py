#!/usr/bin/env python
import  sys
sys.path.append("../")
import numpy as np
import rogata_library as rgt
import rospy

visibility_area = np.load('visibility_box.npy')
contour_array   = [visibility_area]
name            = "visibility_test"
hole_spec       = np.array([1])

obj = rgt.game_object(name,contour_array,hole_spec)



if __name__ == "__main__":
    try:
        rospy.init_node("rogata_engine",anonymous=True)
        rgt.scene([obj])
    except rospy.ROSInterruptException:
        pass

