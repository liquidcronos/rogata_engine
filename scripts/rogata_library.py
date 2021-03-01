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
    get_position()
        returns the position of the objects center
    move_object(new_position,new_ori)
        moves the object to a new position and orientation
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

    def line_intersect(self,start,direction,length,iterations=100,precision=0.001):
        """calculates the intersection between a line and the border of the object

        Iterations and precision are kept at standart values if non are provided
        
        Parameters
        ----------
        start : numpy array
            a 2D point which specifies the start of the line
        direction : numpy array
            a normalized vector specifiying the direction of the line
        length : scalar
            a scalar specifiying the maximum length of the line
        iterations : scalar
            the number of iterations for the ray marching algorithm used
        precision : scalar
            the precision with which the intersection is being calculated

        Returns
        -------
        numpy array
            2D position of the intersection

        """
        position = start
        default  = start+length*direction/np.linalg.norm(direction)
        for k in  range(iterations):
            shortest_dist = self.shortest_distance(position)
            position      = position + shortest_dist*direction/np.linalg.norm(direction)

            if np.linalg.norm(position-start) >= length:
                break
            if shortest_dist <= precision:
                default=position
                break
            
        return default

    def get_position(self):
        """returns the position of the objects center

        The center in this case refers to the mean position of the object.
        For a disjointed area this center can be outside of the object itself.

        Returns
        -------
        numpy array
            2D position of the objects center
        """
        cx   = 0
        cy   = 0
        size = 0
        for contours in self.area:
            moments  = cv2.moments(contours)
            cx   = cx   + moments['m10']
            cy   = cy   + moments['m01']
            size = size + moments['m00']
        return np.array([int(cx/size),int(cy/size)])

    def move_object(self,new_pos,rotate=0):
        """moves the object to a new position and orientation

        Parameters
        ----------
        new_pos : numpy array
            new 2D position of the object
        rotate : scalar
            angle of rotation
        """
        current_center   = self.get_position()
        for i in range(len(self.area)):
            centered_contour = self.area[i] - current_center
            #TODO rotation of contour goes here. Uses polar conversion
            self.area[i]=centered_contour+new_pos

class dynamic_object(game_object):
    def __init__(self,name,position,hitbox):
        #have marker ID OR Color, use move_object function to move
        return 0


class scene():
    
    def __init__(self,game_object_list):
        self.game_objects={}
        for objects in game_object_list:
            self.game_objects[objects.name]= objects

        pos_serv    = rospy.Service('get_position'  ,RequestPos   ,self.handle_get_position)
        dist_serv   = rospy.Service('get_distance'  ,RequestDist  ,self.handle_get_distance)
        inters_serv = rospy.Service('intersect_line',RequestInter ,self.handle_line_intersect)
        inside_serv = rospy.Service('check_inside'  ,CheckInside  ,self.handle_inside_check)
        rospy.spin()


    def handle_get_position(self,request):
        choosen_object = self.game_objects[request.object]
        point          = np.array([reqest.x,request.y])
        pos            = self.game_objects[choosen_object].get_position(point)
        return RequestPosResponse(pos[0],pos[1])

    def handle_line_intersect(self,request):
        choosen_object = self.game_objects[request.object]
        origin         = np.array([request.line.x,request.line.y])
        direction      = np.array([np.cos(request.line.theta),np.sin(request.line.theta)])
        length         = request.length
        intersect      = choosen_object.line_intersect(origin,direction,length)
        return RequestInterResponse(intersect[0],intersect[1])

    def handle_get_distance(self,request):
        choosen_object = self.game_objects[request.object]
        point          = np.array([reqest.x,request.y])
        dist           = self.game_objects[choosen_object].shortest_distance(point)
        return RequestDistResponse(dist)

    def handle_inside_check(self,request):
        choosen_object = self.game_objects[request.object]
        point          = np.array([request.x,request.y])
        inside         = bool(choosen_object.is_inside(point))
        return CheckInsideResponse(inside)

    
def detect_area(hsv_img,lower_color,upper_color,marker_id,min_size,draw=False):
    """Detects the contour of an object containing a marker based on color

    It always returns the smallest contour which still contains the marker
    The contour is detected using an image with hsv color space to be robust under different lighting conditions.
    If draw=True the systems draws all found contours as well as the current smalles one containing the marker onto hsv_img
    

    Parameters
    ----------
    hsv_image : numpy array
        a Image in hsv color space in which the contours  should be detected
    lower_color : numpy array
        a 3x1 array containing the lower boundary for the color detection
    upper_color : numpy array
        a 3x1 array containing the upper boundary for the color detection
    marker_id : scalar
        the ID of a 4x4 aruco marker which identifies the object


    Returns
    -------

    """

    # color detection
    if lower_color[0] <=0:
        second_lower    = lower_color
        second_lower[0] = 179+lower_color[0]
        second_upper    = upper_color
        second_upper[0] = 179

        lower_color[0] = 0
        
        mask1 =cv2.inRange(hsv_img,lower_color,upper_color)
        mask2 =cv2.inRange(hsv_img,second_lower,second_upper)
        mask= mask1 | mask2
    else:
        mask =cv2.inRange(hsv_img,lower_color,upper_color)

    #TODO carefull depending on opencv version the return may be different
    _,contours, hierachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    

    #marker detection:
    gray       = cv2.cvtColor(hsv_img,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, ids=marker_id)
    
    if draw == True:
        cv2.drawContours(hsv_img, contours, -1, (0,255,255),3)
    try:
        if ids.any() == None:
            return None
    except:
        print("No Markers in this Image.")
        print("Without Markers the Calibration can not be performed")
        return None
    if marker_id not in ids:
        return None
    else:
        indice=np.where(ids == marker_id)
        center = np.sum(corners[indice[0][0]][indice[1][0]],axis=0)/4


    if draw == True:
        cv2.circle(hsv_img,(center[0],center[1]),7,(90,255,255),7)

    #TODO smallest contour should be real contour encompassing whole image
    row, col =hsv_img.shape[:2]
    smallest_contour = np.array([[0,0],[0,row],[col,row],[col,0]])
    #TODO not needet with real contour
    contour_found    = 0
    for i in range(len(contours)):
        marker_in_contour = True

        for elements in corners[indice[0][0]][indice[1][0]]:
            marker_in_contour = marker_in_contour and cv2.pointPolygonTest(contours[i],tuple(elements),False) > 0
        marker_in_contour = marker_in_contour and cv2.contourArea(contours[i]) >= min_size
        if  marker_in_contour:
            if cv2.contourArea(contours[i]) <= cv2.contourArea(smallest_contour):
                contour_found = 1
                smallest_contour = contours[i]

    if contour_found == 1:
        if draw == True:
            cv2.drawContours(hsv_img, smallest_contour, -1, (90,255,255),6)
        return smallest_contour

    return None



