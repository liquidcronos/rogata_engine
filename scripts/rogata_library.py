import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from rogata_engine.srv import *

class game_object:
    """
    A class defining the most basic game objects of the engine

    ...

    Attributes
    ----------
    name : str
        the name of the object
    area :  array
        array containing all borders of the object
    holes : array
        array specifying witch border constitues a hole and which an outer border

    Methods
    -------
    is_inside(point)
        checks wheter a point is inside the area of the object
    shortest_distance(point)
        calculates the shortest distance between the point and the border of the object
    line_intersect(start,direction,length,iterations=100,precision=0.01)
        calculates the intersection between a line and the border of the object
    """


    def __init__(self,name,contour_array,hole_spec):
        """
        Parameters
        ----------
        name : str
            the name of the object
        area :  array
            array containing all borders of the object
        holes : array
            array specifying witch border constitues a hole and which an outer border
        """


        #TODO input validation: hole_spec should have length of contour array
        self.name = name
        self.area=contour_array
        self.holes=hole_spec


    def is_inside(self,point):
        """Checks wheter a point is inside the area of the object

        A point directl on the border is also considered inside

        Parameters
        ---------
        point : numpy array
            A 2D point which is to be checked

        
        Returns
        -------
        bool 
            a truthvalue indicating wheter or not the point  is inside the game object 
        """


        #TODO use the hole_spec tree to build the needet logic statement for detemining inside and outside of an arrea
        point=tuple(point)
        
        inside_contour=np.zeros(len(self.area))
        for i in range(len(self.area)):
            inside_contour[i]= cv2.pointPolygonTest(self.area[i],point,False) != -1
            print(inside_contour[i])

        inside = False
        for i in range(min(np.abs(self.holes)),max(np.abs(self.holes))+1):
            holes =  np.argwhere(self.holes == -i)
            areas =  np.argwhere(self.holes ==  i)
            
            inside_hole = False
            for hole in holes:
                inside_hole = inside_hole or inside_contour[hole]

            inside_area = False
            for area in areas:
                inside_area = inside_area or inside_contour[area]

            inside = inside or (inside_area and not inside_hole)

        return bool(inside)

    def shortest_distance(self,point):
        """calculates the shortest distance between the point and the border of the object

        Also returns a positive distance when inside the object

        Parameters
        ---------
        point : numpy array
            A 2D point which is to be checked

        Returns
        -------
        scalar
            distance to border of the object
        """

        point=tuple(point)
        min_dist = np.inf
        for i in range(len(self.area)):
            min_dist=np.minimum(np.abs(cv2.pointPolygonTest(self.area[i],point,True)),min_dist)
        return min_dist

    def line_intersect(self,start,direction,length,iterations=100,precision=0.01):
        touched  = False
        position = start
        for k in  range(iterations):
            shortest_dist = self.shortest_distance(position)
            position      = position + shortest_dist*direction

            if shortest_dist <= precision or np.linalg.norm(position-start) >= length:
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
        #have marker ID OR Color, use move_object function to move
        return 0


class scene():
    
    def __init__(self,game_object_list):
        self.game_objects={}
        for objects in game_object_list:
            self.game_objects[objects.name]= objects

        pos_serv    = rospy.Service('get_position',RequestPos,self.handle_get_position)
        dist_serv   = rospy.Service('get_distance',RequestDist,self.handle_get_distance)
        inters_serv = rospy.service('intersect_line',RequestInter,self.handle_line_intersect)
        inside_serv = rospy.Service('check_inside',CheckInside,self.handle_inside_check)
        rospy.spin()


    def handle_get_position(self,request):
        choosen_object = request.object
        point          = np.array([reqest.x,request.y])
        pos            = self.game_objects[choosen_object].get_position(point)
        return RequestPosResponse(pos[0],pos[1])

    def handle_line_intersect(self,request):
        choosen_object = request.object
        origin         = np.array(request.line.x,request.line.y)
        direction      = np.array([np.cos(request.line.theta),np.sin(request.line.theta)])
        length         = request.line.length
        intersect      = choosen_object.line_intersect(origin,direction,length)
        return RequestInterResponse(intersect[0],intersect[1])

    def handle_get_distance(self,request):
        choosen_object = request.object
        point          = np.array([reqest.x,request.y])
        dist           = self.game_objects[choosen_object].shortest_distance(point)
        return RequestDistResponse(dist)

    def handle_inside_check(self,request):
        choosen_object = request.object
        point          = np.array([reqest.x,request.y])
        inside         = bool(choosen_object.is_inside(point))
        return CheckInsideResponse(inside)

    

