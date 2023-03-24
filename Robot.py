from gpiozero import LED
from gpiozero import Motor
from gpiozero import Servo
from gpiozero import RGBLED
from gpiozero import DistanceSensor
from colorzero import Color
from picamera import PiCamera
from signal import pause
import sys
import os
import pathlib
import time
import board
import busio
import adafruit_bno055
import cv2
import apriltag
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt 
# import pandas as pd
import datetime

#motor pins
ain1, ain2, ena = (5,6,19)
bin1, bin2, enb = (23,24,18)

#rgb led pins
rgb_r, rgb_g, rgb_b = (16, 20, 21)

#i2c sensors
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)


class Robot(object):
    def __init__(self, heading =0, location = (0.0,0.0), speed=0.0):
        self.__location = location
        self.__motor1 = Motor(ain1,ain2,enable=ena)
        self.__motor2 = Motor(bin1,bin2,enable=enb)
        self.__motor3 = Motor(cin1, cin2, enable=enc)
        self.__motor4 = Motor(din1, din2, enable=end)
        self.__timestamp = datetime.datetime.utcnow().timesatamp()
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__imu = adafruit_bno055.BNO055_I2C(self.__i2c)
        self.__rgb_led = RGBLED(rgb_r,rgb_b,rgb_g)
        self.__rgb_led.color = (0, 0, 1)

        self.__pitch = 0.0
        self.__roll = 0.0
        self.__yaw = 0.0
        self.__speed = speed
        self.__history = list()

        #MAX values:
        self.__MAX_SPEED_PERCENT = 100

    def update_state(self, dt):
        self.__timestamp += dt

    def set_indicator_color(self, color):
        self.__rgb_led.color = color

    def set_motor_speed(self, motor, motor_speed=0):
        speed = float(abs(motor_speed))/100.0
        if(motor_speed > 0):
            motor.forward(speed)
        elif(motor_speed < 0):
            motor.reverse(speed)
        else:
            motor.stop()

    def get_position(self):
        return self.__position

    def get_heading(self):
        return self.__heading
    
    def get_speed(self):
        return self.__speed
    
    def get_position_from_apriltags(self):
        #TODO implement functionality!
    

