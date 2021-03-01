#!/usr/bin/env python
import  sys
sys.path.append("../")
import numpy as np
import rogata_library as rgt
import rospy

white_hole    = np.load('white_hole.npy')
right_rock    = np.load('right_rock.npy')
left_rock     = np.load('left_rock.npy')

contour_array_w = [white_hole]
name_w          = "white hole"
hole_spec_w     = np.array([1])

contour_array_l = [left_rock]
name_l          = "left rock"
hole_spec_l     = np.array([1])


contour_array_r = [right_rock]
name_r          = "right rock"
hole_spec_r     = np.array([1])

white_hole_obj = rgt.game_object(name_w,contour_array_w,hole_spec_w)
left_rock_obj  = rgt.game_object(name_l,contour_array_l,hole_spec_l)
right_rock_obj = rgt.game_object(name_r,contour_array_r,hole_spec_r)



if __name__ == "__main__":
    try:
        rospy.init_node('rogata_engine',anonymous=True)
        ray_casting_scene =rgt.scene([white_hole_obj,left_rock_obj,right_rock_obj])
    except rospy.ROSInterruptException:
        pass



