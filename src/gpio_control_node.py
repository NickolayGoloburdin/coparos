#!/usr/bin/env python3
import rospy
from coparos.msg import DroneInfo
from sensor_msgs.msg import NavSatFix 
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
led_pin = 7
class InfoGetter:
    def __init__(self):
        self.status_sub = rospy.Subscriber("/droneInfo", DroneInfo, self.status_callback)
        self.telem_sub = rospy.Subscriber("/gps", NavSatFix, self.telem_callback)
        self.camera_sub = rospy.Subscriber("/camera_alive", Bool, self.camera_callback)
        self.status = False
        self.telem = False
        self.camera = False
    def telem_callback(self,data):
        self.telem = True
    def status_callback(self,data):
        self.status = True
    def camera_callback(self,data):
        self.camera = True
    def check_state(self):
        if self.telem == True and self.status == True and self.camera == True:
            return 0
        elif self.telem == True or self.status == True or self.camera == True:
            return 1
        else:
            return 2
class GPIO_controller:
    def __init__(self, in_pins = [], out_pins = []):
        GPIO.setmode(GPIO.BOARD)
        for i in in_pins:
            GPIO.setup(i, GPIO.IN)
        for i in out_pins:
            GPIO.setup(i, GPIO.OUT)
        self.prev_value = None
    def enable_pin(self,pin):
        GPIO.output(pin, GPIO.HIled_pinGH)
    def disable_pin(self,pin):
        GPIO.output(pin, GPIO.LOW)
    def inverse_pin(self,pin):
        if GPIO.input(pin) == GPIO.HIGH:
            GPIO.output(pin, GPIO.LOW)
        else:
            GPIO.output(pin, GPIO.HIGH)


if __name__ == '__main__':
    rospy.init_node('GPIO_controller')
    infoget = InfoGetter()
    controller = GPIO_controller(out_pins=[led_pin])
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output
    GPIO.output(led_pin, GPIO.LOW)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        state = infoget.check_state()
        if state == 0:
            controller.enable_pin(led_pin)
        elif state == 1:
            controller.inverse_pin(led_pin)
        elif state == 2:
            controller.disable_pin(led_pin)
        rate.sleep()
