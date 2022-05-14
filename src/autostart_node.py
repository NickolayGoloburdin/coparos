#!/usr/bin/env python3
import roslaunch
import rospy
from std_msgs.msg import Float64, String


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jetson/copa5/ws/src/image_matching/image_processing/launch/find_pose.launch"])
class Start:
    def __init__(self,baseAltitude):
        self.baseAltitude = baseAltitude
        self.status = False
        # self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(self.uuid)
        # self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path])
        self.sub_altitude = rospy.Subscriber("/baro",Float64, self.gps_cb, queue_size=10)
        self.log_pub = rospy.Publisher("/logging_topic", String, queue_size=1)
        self.baro_realtive_pub = rospy.Publisher("/baro_relative", Float64, queue_size=1)
    def gps_cb(self,data):
        msg = Float64()
        msg.data = data.data - self.baseAltitude
        self.baro_realtive_pub.publish(msg)
        if abs(data.data - self.baseAltitude) > 20:
            self.setStatus(True)
        else:
            self.setStatus(False)
    def systemStart(self):
        global launch
        launch.start()
    def systemStop(self):
        global launch
        launch.shutdown()
    def setStatus(self,state):
        if self.status != state:
            if state == True:
                self.systemStart()
            else:
                self.systemStop()
            self.status = state
#"/home/haier/catkin_ws/src/testapi/launch/test_node.launch"


if __name__ == "__main__":

    rospy.init_node('AutoStartNode', anonymous=True)
    baro = rospy.wait_for_message("/baro", Float64)
    node = Start(baro.data)
    rospy.loginfo("AutoStart enabled")
    rospy.spin()
# 3 seconds later
