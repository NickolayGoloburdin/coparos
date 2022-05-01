#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
# Структура точки полетного задания



class RemapClass:
    def __init__(self):
        self.pub_gps = rospy.Publisher(
            "/gps", NavSatFix, queue_size=10)
        self.pub_imu = rospy.Publisher(
            "/imu", Imu, queue_size=10)
        self.pub_baro = rospy.Publisher(
            "/baro", Float64, queue_size=10)
        self.gps_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.handle_gps)
        self.imu_sub = rospy.Subscriber(
            "/mavros/Imu/data", Imu, self.handle_imu)
        self.baro_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.handle_baro)

    def handle_gps(self, msg):  
        self.pub_gps.publish(msg)
    def handle_imu(self, msg):  
        self.pub_imu.publish(msg)
    def handle_baro(self, msg): 
        self.pub_baro.publish(msg)

if __name__ == '__main__':
    rospy.init_node('Remap_node')
    RemapClass()
    rospy.spin()
