#unused
from Servo_Motors import ServoMotor 
from gpiozero import LED
from gpiozero import Motor
from gpiozero import Servo
from gpiozero import RGBLED
from gpiozero import DistanceSensor
from gpiozero import Button
from colorzero import Color
from gpiozero import Servo 
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


class Robot(object):
    def __init__(self, 
                heading =0, 
                location = (0.0,0.0,0.0), 
                orientation=(0.0,0.0,0.0),
                speed=0.0,
                left_front_motor_pins=(15,14,18), 
                left_back_motor_pins=(7,8,12), 
                right_front_motor_pins=(5,6,13), 
                right_back_motor_pins=(26,20,19), 
                left_intake_motor_pins=(9,11,10), 
                right_intake_motor_pins=(27,17,22),
                rgbLED_pins = (23,24,25),
                button_pin = 4,
                distance_sensor_left_pin = (0,1),
                distance_sensor_right_pin = (21,16),
                servo_pin_1 = 9,
                servo_pin_2 = 10
                ):

        self.__timestamp = datetime.datetime.utcnow().timestamp()
        self.__location = location
        self.__orientation = orientation

        #Initialize devices
        self.__left_front_motor = Motor(forward= left_front_motor_pins[0], backward= left_front_motor_pins[1], enable= left_front_motor_pins[2], pwm=True) #left front
        self.__left_back_motor = Motor(forward= left_back_motor_pins[0], backward= left_back_motor_pins[1], enable= left_back_motor_pins[2], pwm=True) #left back
        self.__right_front_motor = Motor(forward= right_front_motor_pins[0], backward= right_front_motor_pins[1], enable= right_front_motor_pins[2], pwm=True) #right front
        self.__right_back_motor = Motor(forward= right_back_motor_pins[0], backward= right_back_motor_pins[1], enable= right_back_motor_pins[2], pwm=True) #right back
        self.__left_intake_motor = Motor(forward= left_intake_motor_pins[0], backward= left_intake_motor_pins[1], enable = left_intake_motor_pins[2], pwm=True) #intake left
        self.__right_intake_motor = Motor(forward= right_intake_motor_pins[0], backward= right_intake_motor_pins[1], enable= right_intake_motor_pins[2], pwm=True) #intake right
        
        self.__rgbLED = RGBLED(rgbLED_pins[0],rgbLED_pins[1],rgbLED_pins[2], active_high=True, pwm=True, initial_value=(1,0,0))
        self.__distance_sensor_left = DistanceSensor(echo=distance_sensor_left_pin[0], trigger=distance_sensor_left_pin[1])
        self.__distance_sensor_right = DistanceSensor(echo=distance_sensor_right_pin[0], trigger=distance_sensor_right_pin[1])
        
        self.__button = Button(button_pin, pull_up = True)
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__imu = adafruit_bno055.BNO055_I2C(self.__i2c)
        
        self.__mount_base_servo = ServoMotor()
        self.__mount_camera_servo = ServoMotor()

        

        self.__speed = speed
        self.__history = list()

        #MAX values:
        self.__MAX_SPEED_PERCENT = 100
    
    def update_state(self, dt):
        self.__timestamp += dt

    def set_rgb_indicator_color(self, color):
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
        pass
    

