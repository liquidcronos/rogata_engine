import rogata_library as rgt
import numpy as np
import unittest

height = 100
width  = 100
outer_contour = np.array([[0,0],[0,height],[width,height],[width,0]], dtype=np.int32)
inner_contour = np.array([[0,0],[0,height/10.0],[width/10.0,height/10.0],[width/10.0,0]], dtype=np.int32)
test_object   = rgt.game_object("test_object",[outer_contour,inner_contour],np.array([1,-1]))




class TestStates(unittest.TestCase):
    def test_is_inside(self):
        outside  = test_object.is_inside(np.array([width/20,height/20]))
        inside   = test_object.is_inside(np.array([width/2,height/2]))
        border   = test_object.is_inside(np.array([width,height]))
        d_border = test_object.is_inside(np.array([0,0]))

        self.assertEqual(outside ,False)
        self.assertEqual(inside  ,True)
        self.assertEqual(border  ,True)
        self.assertEqual(d_border,False)

    def test_get_position(self):
        center      = test_object.get_position()
        true_center = [49,49]

        self.assertTrue((center == true_center).all())
# Test basic functionality
#name (test name not string (should throw error))
#get_position

#line_intersect (test required precision, test if no interesction, test not normalized direction)
#move_object     (test new position, test rotation, test rotation angle with different conventions (larger than pi, -pi))
#shortest_distance (test distance inside,test distance outside, test point on object border, )


if __name__ == '__main__':

    unittest.main()

