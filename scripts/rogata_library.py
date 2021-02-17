import numpy as np
import cv2
import cv2.aruco as aruco

class game_object:
    def __init__(self,name,contour_array,hole_spec):
        #TODO input validation: hole_spec should have length of contour array
        self.name = name
        self.area=contour_array
        self.holes=hole_spec

    def __del__(self):
        print('Object deleted')

    def is_inside(self,point):
        #TODO use the hole_spec tree to build the needet logic statement for detemining inside and outside of an arrea

        for i in range(len(self.array)):
            #Checks if inside
            inside_contour=cv2.pointPolygonTest(self.area[i],point,False)
        return 0

    def shortest_distance(self,point):
        min_dist = np.inf
        for i in range(len(self.array)):
            min_dist=np.min(np.abs(cv2.pointPolygonTest(self.area[i],point,True)),min_dist)
        return min_dist

    def line_intersect(self,start,direction,length):
        for k in  np.arange(length,1):
            touched= False
            position=start+k*direction
            for i in range(len(self.array)):
                touched=touched or (cv2.pointPolygonTest(self.area[i],point,False)!=1)
            if touched != False:
                break
        return position
