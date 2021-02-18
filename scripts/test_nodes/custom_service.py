#!/usr/bin/env python
import rospy
from rogata_engine.srv import RequestPos, RequestPosResponse



def handle_service(req):
    print(req)
    return RequestPosResponse(1,0)


def request_position_server():
    rospy.init_node('request_position_server')
    serv = rospy.Service('requestPos',RequestPos,handle_service)
    rospy.spin()


if __name__=="__main__":
    request_position_server()
