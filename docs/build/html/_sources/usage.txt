Getting Started
***************

.. toctree::
   :hidden:
   



The RoGaTa engine build upon `Ros Melodic`_ to interface with the robots, and `OpenCV-Python`_ to calibrate the arena and track all dynamic objects.
The first step is thus to install ROS on all robots as well as the host PC. 

The host PC refers in this case to the PC which runs the engine which means that additonally OpenCV has to be installed there.





.. _Ros Melodic: http://wiki.ros.org/melodic
.. _OpenCV-Python: https://docs.opencv.org/master/da/df6/tutorial_py_table_of_contents_setup.html




Setting up the Enviroment
=========================
Once ROS and OpenCV are installed the environment of the engine has to be set up.
First a catkin workspace has to be created.
For this `catkin tutorial`_ can be followed.

The RoGaTa Engine itself is a ROS package containing nodes for camera based sensing and a library that provides utilities for game development.
The package has to be placed inside the ``catkin_ws/src`` directory::

    cd ~/catkin_ws/src
    git clone https://github.com/liquidcronos/RoGaTa-Engine

Afterward the ``catkin_make`` command has to be called inside the ``catkin_ws`` directory::

    cd ..
    catkin_make

If everything is correctly installed, the following command should change the current directory to the one containing the ROS package::

    roscd rogata_engine

.. _catkin tutorial: http://wiki.ros.org/catkin/Tutorials/create_a_workspace



Setting up a Scene
==================
Scenes can be thought of as real life video game levels.
A Scene is made up of multiple game objects whose position is determined by a camera above the physical level.
Setting up a scene thus simply means getting the engine to recognize phyiscal objects as game objects.

The RoGaTa engine uses a simple recognition based on an objects color.

