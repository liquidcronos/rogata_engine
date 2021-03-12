Tutorials
************

.. toctree::
   :hidden:
   




Simple Scoreboards
==================

Scoreboards a staple of Video games, defining what constitues good play.
In terms of machine learning they also have uses for calculating an agents fitness.

This simple tutorial will give an example of how to implement such a scoreboard.
It will assume that the game scene is already set up and can be accessed.
For the purposes of this example the scene will include a number of coins that the player needs to collect.
Since each coin is independent from the rest, each coin is its own game objects and will follow the naming convention 
``coin/i`` where ``i`` is a integer smaller or equal to 12.

The complete Scoreboard code now looks like this:
::
    
    #!/usr/bin/env python
    import numpy as np
    import rogata_library as rgt
    from rogata_engine.srv import *
    import rospy

    class coin_scoreboard():

        def __init__(self,player_list,coin_name,coin_number):
            self.player_list = player_list
            self.coin_name   = coin_name
            self.coin_number = coin_number
            self.collection  = {}

            self.check_inside   = rospy.ServiceProxy('check_inside',CheckInside)

            for players in player_list:
                self.collection[players] = np.zeros(self.coin_number)
                rospy.Subscriber(players+"/odom",self.manage_score,players)

            rospy.spin()

        def manage_score(self,data,player):
            point = np.array([data.pose.pose.position.x,data.pose.pose.position.x])
            already_collected = self.collection[player]
            for i in range(self.coin_number):
                req  = CheckInsideRequest("coint/"+str(i),point[0],point[1])
                resp = self.check_inside(req)

                already_collected[i] = already_collected[i] or resp.inside

            self.collection[player] = already_collected
            rospy.set_param(player+"/score",np.sum(already_collected))


    if __name__ = "__main__":
        rospy.init_node("score_board")
        score = coin_scoreboard(["player_1","player_2"],"coin",12)


This code might seem complex, the following sections will therefore break it down line by line.
A first question one might ask beforehand is why a class is needet to implement this scoreboard.
The reason for this is that it is not trivial to store information from a subscriber outside its `callback function <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_.
Class variables are simply a convenient way to circumvent this problem.

Getting back to the code, the line
::
    
    #!/usr/bin/env python

Is needet for every Python ROS Node. This first line makes sure that the script is exectured with Python.
::

    import numpy as np
    import rogata_library as rgt
    from rogata_engine.srv import *
    from nav_msgs.msg import Pose2D
    import rospy

These lines declare the needed libraries and modules. Numpy offers convenient array math functions and rospy the ROS interface. ``rogata_engine.srv`` and ``Pose2D`` are needet to import the `ROS service <http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29>`_ objects needet to call the RoGaTa engine services.
::

        def __init__(self,player_list,coin_name,coin_number):
            self.player_list = player_list
            self.coin_name   = coin_name
            self.coin_number = coin_number
            self.collection  = {}

            self.check_inside   = rospy.ServiceProxy('check_inside',CheckInside)

            for players in player_list:
                self.collection[players] = np.zeros(self.coin_number)
                rospy.Subscriber(players+"/odom",Pose2D,self.manage_score,players)

            rospy.spin()

The ``__init__`` function initializes the class. In this case this requires a list of player names, the name of the coin object and the coin number.
The last two arguments could also have been hardcoded, but are provided here as inputs to make the class more flexible.
The fuction also initializes the dictionary ``self.collection`` which keeps track of which coins have already been collected by a given player.
It is populated inside the foor loop.
Here also a `Subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_ is setup for each player which reads in the position of each player.
For this to work, the names of the players have to refer to dynamic objects initialized in a :py:class:`rogata_library.scene` object.

Additionally a service ``self.check_inside`` is set up to request information from the engine wheter a given point is inside a object.
This will later be used to check wheter a player has collected a ``coin``.

Lastly ``rospy.spin()`` is called, which ensures that the system stayes active and does not imediately terminate.
::

        def manage_score(self,data,player):
            point = np.array([data.pose.pose.position.x,data.pose.pose.position.x])
            already_collected = self.collection[player]
            for i in range(self.coin_number):
                req  = CheckInsideRequest("coint/"+str(i),point[0],point[1])
                resp = self.check_inside(req)

                already_collected[i] = already_collected[i] or resp.inside

            self.collection[player] = already_collected
            rospy.set_param(player+"/score",np.sum(already_collected))

The ``manage_score`` function is the callback of the position subscriber. This means it gets called every time the position of the dynamic objects are updated.
The returned argument ``data`` is of type ``Pose2D`` and first has to be converted to a point.
Using the current position of a player, the list of collected coints can now be updated. For each coin the foor loop calls the ``self.check_inside`` service to check if a new coin has been collected.
The sum of all collected coints is lastly set as a `ROS parameter <http://wiki.ros.org/Parameter%20Server>`_ 
This allows any other ROS node to check the score of a given player with name ``PLAYERNAME`` by calling
::

    rospy.get_param(PLAYERNAME/score)


Simple Line of Sight Calculation
================================
Another Common Problem is line of sight calculation.
While robots can do line of sight calcualtion with an on board camera this requires object detection.
The RoGaTa engines, allows to circumvent these requirements by using line interesection.
This allows the design of stealth like games where a ``thief`` has to enter a area undetected by one or more ``guards``.
To calulate wheter a ``guard`` can see a ``thief`` the following function can be used:
::

    import numpy as np
    import rogata_library as rgt

    rogata = rgt.rogata_helper()

    def visibility(guard,thief,wall_objects,seeing_distance):
        distance   = np.linalg.norm(thief-guard)
        dir_vector = -guard+thief
        direction  = np.arctan2(dir_vector[1],dir_vector[0])

        seeing_distance = seeing_distance
        for walls in wall_objects:
            seeing_distance = np.min(seeing_distance,rogata.intersect(walls,guard,direction,seeing_distance))
        if np.linalg.norm(seeing_distance-guard) >= distance:
            return 1
        else:
            return 0

Here the ``rogata_helper`` class is used in order to abstract the ``get_intersection`` service of the engine.
The Function defines a line between ``guard`` and ``thief`` and checks if this line is intersect by an object that is not see through.
Since multiple such ``walls`` could exist, the system accepts a list called ``wall_objects``.
If there is an intersection beween the ``guard`` and the ``thief`` the line of sight is broken and the function returns ``False``.
Otherwise the two see each other and the function returns ``True``.



