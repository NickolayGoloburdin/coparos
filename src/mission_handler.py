#!/usr/bin/env python3
import rospy
from coparos.msg import MissionPoint as MissionPointMsg
from coparos.srv import Service_command, Load_mission, Service_commandResponse, Load_missionResponse
from std_msgs.msg import Int16

class MissionPoint:
    def __init__(self):
        self.targetLat = 0
        self.targetLon = 0
        self.targetAlt = 0
        self.targetRadius = 0
        self.loiterTime = 0
        self.maxHorizSpeed = 0
        self.maxVertSpeed = 0
        self.poiLat = 0
        self.poiLon = 0
        self.poiHeading = 0
        self.poiAltitude = 0
        self.flags = 0
        self.photo = 0
        self.panoSectorsCount = 0 
        self.panoDeltaAngle = 0
        self.poiPitch = 0
        self.poiRoll = 0
        self.type = 0

class MissionHandler:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/MissionPoint", MissionPointMsg, queue_size=10)
        self.mission_request_handler = rospy.Subscriber("/mission_request", Int16, self.handle_request)
        self.load_mission_service = rospy.Service("/LoadMissionFromJson", Load_mission, self.load_mission_fron_json)
        self.reset_service = rospy.Service("/SendMission", Service_command, self.send_mission)
        self.points = []
    def handle_request(self, msg):
        i = msg.data
        if len(self.points) == 0:
            rospy.logerr("Mission is empty")
            return False
        if msg.data < len(self.points):
            msg = create_msg_point(i)
            self.pub.publish(msg)

    def create_msg_point(self,i):
        msg = MissionPoint()
        msg.targetLat = self.points[i].targetLat
        msg.targetLon = self.points[i].targetLon
        msg.targetAlt = self.points[i].targetAlt
        msg.targetRadius = self.points[i].targetRadius
        msg.loiterTime = self.points[i].loiterTime
        msg.maxHorizSpeed = self.points[i].maxHorizSpeed
        msg.maxVertSpeed = self.points[i].maxVertSpeed
        msg.poiLat = self.points[i].poiLat
        msg.poiLon = self.points[i].poiLon
        msg.poiHeading = self.points[i].poiHeading
        msg.poiAltitude = self.points[i].poiAltitude
        msg.flags = self.points[i].flags
        msg.photo = self.points[i].photo
        msg.panoSectorsCount = self.points[i].panoSectorsCount
        msg.panoDeltaAngle = self.points[i].panoDeltaAngle
        msg.poiPitch = self.points[i].poiPitch
        msg.poiRoll = self.points[i].poiRoll
        msg.type = self.points[i].type
        return msg
    def send_mission(self, req):
        if len(self.points) == 0:
            rospy.logerr("Mission is empty")
            return False
        msg = create_msg_point(0)
        self.pub.publish(msg)
        

    def load_mission_fron_json(self, req):
        return True
if __name__ == '__main__':
    rospy.init_node('Mission_handler')
    realtime = rospy.get_param("/realtime")
    if not realtime:
        exit()
    MissionHandler()
    rospy.spin()
