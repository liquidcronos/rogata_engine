# Robokart-Engine

The Robotic Games Tabletop engine is a set of ROS based software tools that bridge the gap between mobile robotics and game development. It uses a camera to identify real objects and areas and iinitialize them as game objects. This allows the engine to offer the same functionality a normal game engine provides.

Example functionalities include: 
* calculate line of sight between robots without on-board cameras 
* check if a robot has entered a specified area 
* simulate laser scanner readings with virtual walls

Using these functionalities it is possible to develop video games which use real robots as actors.

## Who this engine is for
Since most games require multiple robots this engine is primarily aimed at educators. The idea being that games can be set up with students developing the ‘ai’ of the npcs thereby familiarzing themself with the development of mobile robots.

However the engine can also be used for experiments with single robots as the virtual game objects allow the simulation of sensor data such as a laser scanner. Therefore the engine might also be usefull to researchers who need to quickly set up multiple arenas for their mobile robots, or might want to use the additional information the enigne provides for a fitness function of a machine learning approach
## Documentation
The Docoumentation can be found on [https://rogata-engine.readthedocs.io/](https://rogata-engine.readthedocs.io/en/latest/what_is_rogata.html) 
