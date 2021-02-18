import cv2
import numpy as np
import rogata_library as rgt

contour       = np.load('contour_save.npy')
contour_array = [contour,contour]
name          = "test object"
hole_spec     = [1,1]

obj = rgt.game_object(name,contour_array,hole_spec)

print(obj.name)
print(obj.get_position())

height=600
width=600
blank_image = np.zeros((height,width,3), np.uint8)
cv2.drawContours(blank_image,obj.area,-1,(0,255,0),3)


test_point= np.array([180,200])
print(obj.shortest_distance(test_point))

for i in range(36):
    direction = np.array([np.sin(2*np.pi/36*i),np.cos(2*np.pi/36*i)])
    length    = 300
    end_point = obj.line_intersect(test_point,direction,length)
    cv2.line(blank_image,tuple(test_point),(int(end_point[0]),int(end_point[1])),(255,0,0),3)


cv2.imshow("Test",blank_image)
k = cv2.waitKey(0)
