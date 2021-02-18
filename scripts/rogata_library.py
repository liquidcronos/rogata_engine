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
        point=tuple(point)

        for i in range(len(self.area)):
            #Checks if inside
            inside_contour=cv2.pointPolygonTest(self.area[i],point,False)
        return 0

    def shortest_distance(self,point):
        point=tuple(point)
        print(point)
        min_dist = np.inf
        for i in range(len(self.area)):
            min_dist=np.minimum(np.abs(cv2.pointPolygonTest(self.area[i],point,True)),min_dist)
        return min_dist

    def line_intersect(self,start,direction,length):
        touched  = False
        for k in  np.arange(0,length,1):
            position = start + k*direction
            for i in range(len(self.area)):
                point   = tuple(position)
                touched = touched or (cv2.pointPolygonTest(self.area[i],point,False)!=-1)
            if touched != False:
                break
        return position

    def get_position(self):
        cx   = 0
        cy   = 0
        size = 0
        for contours in self.area:
            moments  = cv2.moments(contours)
            cx   = cx   + moments['m10']
            cy   = cy   + moments['m01']
            size = size + moments['m00']
        return np.array([int(cx/size),int(cy/size)])

    def move_object(self,new_pos,new_ori=0):
        current_center   = self.get_position()
        print(current_center)
        for i in range(len(self.area)):
            centered_contour = self.area[i] - current_center
            #TODO rotation of contour goes here. Uses polar conversion
            self.area[i]=centered_contour+new_pos
        return 0

class dynamic_object(game_object):
    def __init__(self,name,position,hitbox):
        #compute contour from hitbox and position
        return 0


