import rogata_library as rgt
import numpy as np
import unittest

height = 100
width  = 100
outer_contour = np.array([[0,0],[0,height],[width,height],[width,0]], dtype=np.int32)
inner_contour = np.array([[0,0],[0,height/10.0],[width/10.0,height/10.0],[width/10.0,0]], dtype=np.int32)
test_object   = rgt.game_object("test_object",[outer_contour,inner_contour],np.array([1]))


# Test basic functionality
#name (test name not string (should throw error))
#get_position
#is_inside  (mind edge case of the point being on the line)
#line_intersect (test required precision, test if no interesction, test not normalized direction)
#move_object     (test new position, test rotation, test rotation angle with different conventions (larger than pi, -pi))
#shortest_distance (test distance inside,test distance outside, test point on object border, )


# Test validity of object hierachy


