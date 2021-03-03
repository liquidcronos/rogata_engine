How it Works
************

.. toctree::
   :hidden:
   

The RoGaTa engine defines game objects by detecting and tracking their position using a camera afixed over the game area.

.. image:: setup.png

The main consideration when choosing an object detection scheme for the engine is the ability to quickly setup the game area and initialize the game objects.
For this reason a mixture of marker based tracking and color based detection was used.
Before explaining in detail how the tracking is performed, one first has to define what exactly needs to be tracked.
For this reason the next section will introduce how a game object is defined.

Game Objects
===========
Game Objects are the basic building blocks of the engine.
Examples of game objects include:
 * robots
 * movable obstacles
 * Areas
 * Buttons
 * Walls

In general such an object is defined by a name aswell as the space it occupies within the game area.
The simplest way to define an area is using a mask witch specifies for each position wheter the point is inside or outside the area.
In robotics such a concept is sometimes also called an occupancy grid.
However since storing and manipulating such grids is resource intensive.
For this reason only the borders of an object are used to define its area.

However borders alone are not enough to fully define an object, since for more complex objects it is not clear what is defined as inside and outside.
This can be illustrated in te following image:

.. image:: complex_object.png

To make the area definition unambiguous, a hierachy can be introduced.
The outermost border of an object has a hieracy of 1, if it has a corresponding inner border it has a hierachy of -1.
Since more complex objects might have another outside border inside a inner border, these are denoted by a hierachy of 2.
The general defintion is as follows:
 1. Outer borders are denoted by positive numbers a
 2. The inner border corresponding to a outer border has a hierachy of -a
 3. The hierachy a is equal to 1+b where b is the hierachy of the smallest border surrounding the considered border

Using the border, which is specified as a opencv contour object aswell as a name and a hierachy a game object can be initialized.
Its documentation can be seen in :py:class`rogata_library.game_object`.


Detecing the object borders
---------------------------
Using a color range specified in HSV values (which are better at detecting colors in different lighting conditions since the Hue of the color is independent from its Saturation and Value) a mask of all objects can be generated which share the specified color.


