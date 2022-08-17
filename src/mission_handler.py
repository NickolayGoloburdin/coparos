#!/usr/bin/env python3
import rospy
import numpy as np
from coparos.msg import MissionPoint as MissionPointMsg
from coparos.srv import Service_command, Load_mission, Service_commandResponse, Load_missionResponse, Download_mission, Download_missionResponse
from std_msgs.msg import Int16
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
# Структура точки полетного задания


class MissionPoint:
    def __init__(self, pList=None):
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
    def __init__(self):
        self.counter = 0
        # Инициализация издателя для отправки точки в коптер
        self.pub = rospy.Publisher(
            "/MissionPoint", MissionPointMsg, queue_size=10)
        # Подписчик на топик запрашиваемой коптером точки
        self.mission_request_handler = rospy.Subscriber(
            "/missionRequest", Int16, self.handle_request)  # Топик ответа на запрос точек от дрона
        self.mission_responce_handler = rospy.Subscriber(
            "/missionResponce", MissionPointMsg, self.handle_responce)  # Топик ответа на точки от дрона
        self.load_mission_service = rospy.Service(
            "/LoadMissionFromFile", Load_mission, self.load_mission_from_file)  # Сервис загрузки миссии из JSON
        self.reset_service = rospy.Service(
            "/SendMission", Trigger, self.send_mission)  # Сервис отправки миссии
        self.clear_service = rospy.Service(
            "/ClearMission_on_jetson", Trigger, self.clear_mission)  # Сервис отправки миссии
        self.download_service = rospy.Service(
            "/GetMissionPointsList", Download_mission, self.get_points)  # ервис загрузки миссии из дрона
        self.points = []

    def handle_responce(self, msg):
        point = MissionPoint()
        point.targetLat = msg.targetLat
        point.targetLon = msg.targetLon
        point.targetAlt = msg.targetAlt
        point.targetRadius = msg.targetRadius
        point.loiterTime = msg.loiterTime
        point.maxHorizSpeed = msg.maxHorizSpeed
        point.maxVertSpeed = msg.maxVertSpeed
        point.poiLat = msg.poiLat
        point.poiLon = msg.poiLon
        point.poiHeading = msg.poiHeading
        point.poiAltitude = msg.poiAltitude
        point.flags = msg.flags
        point.photo = msg.photo
        point.panoSectorsCount = msg.panoSectorsCount
        point.panoDeltaAngle = msg.panoDeltaAngle
        point.poiPitch = msg.poiPitch
        point.poiRoll = msg.poiRoll
        point.type = msg.type
        self.points.append(point)

    def handle_request(self, msg):  # Метод обработчки сообщения запрашиваемой точки
        i = msg.data
        # Если запрашиваемая точка есть в массиве точек из JSON то она отправляется в коптер
        if len(self.points) == 0:
            rospy.logerr("Mission is empty")
            return False
        if msg.data < len(self.points):
            msg = self.create_msg_point(i)
            self.pub.publish(msg)

    def create_msg_point(self, i):
        # Инициализация сообщения с запрашиваемой точкой
        msg = MissionPointMsg()
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
        # Метод начала загрузки полетного задания в коптер

    def send_mission(self, req):

        if len(self.points) == 0:
            rospy.logerr("Mission is empty")
            return TriggerResponse(False)
        msg = self.create_msg_point(0)
        self.pub.publish(msg)
        return TriggerResponse(True)

    def clear_mission(self, req):
        self.points = []
        return TriggerResponse(True)

    def load_mission_from_file(self, req):
        file_name = "/home/jetson/copa5/missions/{}.BIN".format(req.number)
        dtype = np.dtype([('lat', np.float64), ('lon', np.float64), ('alt', np.float32), ('r', np.float32), ('time', np.int32), ('hs', np.float32),
                          ('vs', np.float32), ('plat', np.float64), ('plon', np.float64), (
                              'ph', np.float32), ('pa', np.float32), ('flags', np.uint32),
                          ('photo', np.uint8), ('psc', np.uint8), ('pda', np.float32), ('pp', np.float32), ('pr', np.float32), ('typ', np.uint8)])

        with open(file_name, "rb") as bin_file:
            num_data = np.fromfile(bin_file, dtype)
        list_num_data = num_data.tolist()
        self.fill_mission_handler(list_num_data)
        return Load_missionResponse(len(self.points))

    def get_points(self, req):
        res = Download_missionResponse()
        for i in range(len(self.points)):
            res.append(self.create_msg_point(i))
        return res

    def fill_mission_handler(self, pointList):
        self.points.clear()
        for i in pointList:
            self.points.append(MissionPoint(i))


if __name__ == '__main__':
    rospy.init_node('Mission_handler')
    MissionHandler()
    rospy.spin()
