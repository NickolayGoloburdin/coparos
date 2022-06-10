#!/usr/bin/env python3
from operator import truediv
from sre_constants import SUCCESS
import rospy
import numpy as np
from coparos.msg import MissionPoint as MissionPointMsg
from coparos.srv import Service_command, Load_mission, Service_commandResponse, Load_missionResponse
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
# Структура точки полетного задания


class MissionPoint:
    def init(self, pList=None):
        if pList == None:
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
        else:
            self.targetLat = pList[0]
            self.targetLon = pList[1]
            self.targetAlt = pList[2]
            self.targetRadius = pList[3]
            self.loiterTime = pList[4]
            self.maxHorizSpeed = pList[5]
            self.maxVertSpeed = pList[6]
            self.poiLat = pList[7]
            self.poiLon = pList[8]
            self.poiHeading = pList[9]
            self.poiAltitude = pList[10]
            self.flags = pList[11]
            self.photo = pList[12]
            self.panoSectorsCount = pList[13]
            self.panoDeltaAngle = pList[14]
            self.poiPitch = pList[15]
            self.poiRoll = pList[16]
            self.type = pList[17]

# Класс для отправки полетных заданий в коптер


class MissionHandler:
    def init(self):
        self.counter = 0
        # Инициализация издателя для отправки точки в коптер
        self.load_mission_service = rospy.Service(
            "/LoadMissionFromFile", Load_mission, self.load_mission_from_file)  # Сервис загрузки миссии из JSON
        self.reset_service = rospy.Service(
            "/SendMission", Trigger, self.send_mission)  # Сервис отправки миссии
        self.points = []

        # Метод начала загрузки полетного задания в коптер

    def prepare_waypoints(self):
        waypoints = []
        pt = Waypoint()
        pt.frame = 0
        pt.is_current = False
        pt.autocontinue = True
        pt.z_alt = self.points[0].targetAlt
        pt.command = 22
        waypoints.append(pt)
        for i in self.points:
            pt.frame = 0
            pt.is_current = False
            pt.autocontinue = True
            pt.x_lat = i.targetLat
            pt.y_long = i.targetLon
            pt.z_alt = i.targetAlt
            pt.command = 16
            waypoints.append(pt)
        pt.frame = 0
        pt.is_current = False
        pt.autocontinue = True
        pt.command = 20
        waypoints.append(pt)
        return waypoints

    def send_mission(self, req):
        res = TriggerResponce()
        wps = self.prepare_waypoints()
        if len(self.points) == 0:
            rospy.logerr("Mission is empty")
            res.message = "Mission is empty"
            res.success = False
            return res
        rospy.wait_for_service('/mavros/mission/push')
        try:
            push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            resp1 = push(0, wps)
            res.success = resp1.success
            res.message = "pushed {} points".format(resp1.wp_transfered)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        res.success = True
        return res

    def load_mission_from_file(self, req):
        file_name = "/home/rodion/missions/{}.BIN".format(req.number)
        dtype = np.dtype([('lat', np.float64), ('lon', np.float64), ('alt', np.float32), ('r', np.float32), ('time', np.int32), ('hs', np.float32),
                          ('vs', np.float32), ('plat', np.float64), ('plon', np.float64), (
                              'ph', np.float32), ('pa', np.float32), ('flags', np.uint32),
                          ('photo', np.uint8), ('psc', np.uint8), ('pda', np.float32), ('pp', np.float32), ('pr', np.float32), ('typ', np.uint8)])

        with open(file_name, "rb") as bin_file:
            num_data = np.fromfile(bin_file, dtype)
        list_num_data = num_data.tolist()
        self.fill_mission_handler(list_num_data)
        return Load_missionResponse(len(self.points))

    def fill_mission_handler(self, pointList):
        self.points.clear()
        for i in pointList:
            self.points.append(MissionPoint(i))


if __name__ == '__main__':
    rospy.init_node('Mission_handler')
    a = MissionHandler()
    rospy.spin()
