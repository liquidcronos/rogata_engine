#!/usr/bin/env python
import rospy
from rogata_engine.srv import RequestPos

client=rospy.ServiceProxy('requestPos',RequestPos)
# call client
print(client('test'))
