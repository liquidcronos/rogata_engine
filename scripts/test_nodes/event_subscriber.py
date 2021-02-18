#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool



def on_event(event,callback):
    class event_handler():
        def __init(self):
            self.event_state = 0
            rospy.Subscriber(event,Bool,self.continuos_callback,callback)


        def continuos_callback(data):
            if data-self.event_state() == 1:
                callback()
                self.event_state= not(self.event_state)
    event_handler(callback)


callback("test",test_callback)


def test_callback():
    print("beeb")
