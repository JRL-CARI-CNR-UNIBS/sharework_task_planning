#! /usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from mqtt_scene_integration.msg import Fixture

GREEN = '\033[92m'
RED = '\033[91m'
END = '\033[0m'


PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
SERVICE_NAME_CHECK = "check_new_piece"
SERVICE_NAME_RESET = "reset_new_piece"


class Station:
    """docstring for ClassName."""
    def __init__(self, ):
        self.state_ = False
        self.content_ = None
    def updateStationState(self,fixture_msg):
        if len(fixture_msg.content)>0:
            self.state_ = True
            self.content_ = fixture_msg.content.replace("A","a")

    def resetPiece(self,req):
        self.state_ = False
        self.content_ = None
        return TriggerResponse(True,self.content_)
    def checkNewPiece(self,req):
        if self.state_:
            response = TriggerResponse(True,self.content_)

        else:
            response = TriggerResponse(False,None)
        return response



def main():
    rospy.init_node('check_new_piece', anonymous=True)
    rospy.loginfo(GREEN + "Check new piece service on" + END)

    try:
        trigger_topic_name=rospy.get_param("/trigger_topic_name")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("/trigger_topic_name") + END)
        return 0

    station_p0 = Station()
    chek_piece_srv = rospy.Service(SERVICE_NAME_CHECK, Trigger, station_p0.checkNewPiece)
    reset_piece_srv = rospy.Service(SERVICE_NAME_RESET, Trigger, station_p0.resetPiece)

    station_p0_sub = rospy.Subscriber(trigger_topic_name,Fixture, station_p0.updateStationState)

    rospy.spin()



if __name__ == "__main__":
    main()
