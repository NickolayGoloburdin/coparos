import roslaunch
import rospy
from std_msgs import Float64
class Start:
    def __init__(self,baseAltitude, path):
        self.baseAltitude = baseAltitude
        self.status = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path])
        self.sub_altitude = rospy.Subscriber("/baro",Float64, self.gps_cb, queue_size=10)
    def gps_cb(self,data):
        if abs(data.data - self.baseAltitude) > 20:
            self.setStatus(True)
        else:
            self.setStatus(False)
    def systemStart(self):
        self.launch.start()
    def systemStop(self):
        self.launch.shutdown()
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
    node = Start(baro.data, "/home/jetson/copa5/ws/src/image_matching/image_processing/launch/find_pose.launch")
    rospy.loginfo("AutoStart enabled")
    rospy.spin()
# 3 seconds later
