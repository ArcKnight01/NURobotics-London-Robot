import os
import pathlib
import sys
if os.uname().nodename == 'robotpi':
    from gpiozero import Motor
    from gpiozero import RGBLED
    from gpiozero import Button
    from gpiozero import DistanceSensor
    from colorzero import Color
import time
import numpy as np
from ADCS_System import *
from Image_Processor import *
from DCMotors import *
from Sonar import Sonar
from Servo_Motors import ServoMotor
from RGB_Indicator import RGB_Indicator
import psutil
import warnings
warnings.filterwarnings('ignore')



class AutonomousController(object):
    def __init__(self,
                # robot_state,
                verbose = False,
                motor1_pins=(15,14,18), 
                motor2_pins=(7,8,12), 
                motor3_pins=(5,6,13), 
                motor4_pins=(26,20,19), 
                motor5_pins=(9,11,10), 
                motor6_pins=(27,17,22),
                rgb_pins = (23,24,25),
                button_pin = 4,
                distance_sensor_left_pin = (0,1),
                distance_sensor_right_pin = (21,16),
                top_servo_pin = 9,
                bottom_servo_pin = 10,
                ):

        self.__heading = None
        self.__desired_heading = None
        self.__rgbLED = RGB_Indicator(enable=True, verbose=False, red_pin=rgb_pins[0], green_pin=rgb_pins[1], blue_pin=rgb_pins[2],pwm=True, initial_color=(255,0,0))
        
        self.__motor1 = DCMotor(verbose=False, enabled=True, pins=motor1_pins, rgbLED=self.__rgbLED)
        self.__motor2 = DCMotor(verbose=False, enabled=True, pins=motor2_pins, rgbLED=self.__rgbLED)
        self.__motor3 = DCMotor(verbose=False, enabled=True, pins=motor3_pins, rgbLED=self.__rgbLED)
        self.__motor4 = DCMotor(verbose=False, enabled=True, pins=motor4_pins, rgbLED=self.__rgbLED)
        self.__motor5 = IntakeMotor(verbose=False, enabled=False, pins=motor1_pins, rgbLED=self.__rgbLED)
        self.__motor6 = IntakeMotor(verbose=False, enabled=False,  pins=motor1_pins, rgbLED=self.__rgbLED)
        
        self.__button = Button(button_pin)
        # self.__distance_sensor_left = DistanceSensor(echo=distance_sensor_left_pin[0], trigger=distance_sensor_left_pin[1])
        # self.__distance_sensor_right = DistanceSensor(echo=distance_sensor_right_pin[0], trigger=distance_sensor_right_pin[1])

        self.__sonar_left = Sonar(verbose=False, enable=True, echo_pin= distance_sensor_left_pin[0], trig_pin=distance_sensor_left_pin[1])
        self.__sonar_right = Sonar(verbose=False, enable=True, echo_pin= distance_sensor_right_pin[0], trig_pin=distance_sensor_right_pin[1])

        # self.distances = self.get_distances()
        # self.ultrasound_enabled = False
        
        
        self.__adcs_system = ADCS(test_points=5, verbose=False, enabled=True)
        self.__image_processor = ImageProcessor('./', verbose=False)
        self.__first_start = True
        self.__start_time = None

        self.__current_time = 0
        
        self.__competition_timer = 0.0
        self.__timer = 0.0
        self.__verbose = verbose
        self.__camera_enabled = True
        
        
        self.__on_state = False #change to false if you want
        self.__button.when_pressed = self.switch_on_state

    def check_if_endgame(self, threshold)->bool:
        if self.__competition_timer >= threshold:
            return(True)
        else:
            return(False)
        
    def switch_ultrasound_enable(self):
        if(self.ultrasound_enabled ==False):
            self.ultrasound_enabled = True
        elif(self.ultrasound_enabled==True):
            self.ultrasound_enabled = False
        return(self.ultrasound_enabled)

    def get_current_time(self):
        return(self.__current_time)
    
    def initialize(self, robot_state):
        #UNUSED
        #initialize heading
        # self.__heading = robot_state['heading']
        # self.__speed = robot_state['speed']
        # self.__position = robot_state['position']
        # self.__desired_heading = robot_state['heading']
        pass
    

    def switch_on_state(self):
        """This function runs whenever the button is pressed"""
        if(self.__first_start):
            self.__competition_start_time = time.time()
            self.__start_time = self.__competition_start_time
            self.__first_start = False
        
        print("BUTTON PRESS DETECTED",end=" ")
        if self.__on_state == True:
            # if(self.__verbose==True):
                # print("TURNING OFF")
            self.__on_state = False
        elif self.__on_state == False:
            # if(self.__verbose==True):
            #     print("TURNING ON")
            self.__start_time = time.time()
            self.__on_state = True
        
        self.__on_state = True
        self.stop_motors()
        time.sleep(1)
    
    def __repr__(self):

        return f"Robot Class"
        
    def drive_motors(self, left_speed:float=0.0, right_speed:float=0.0):
        self.run_motor(self.__motor1, left_speed, "fwd")
        self.run_motor(self.__motor2, left_speed, "fwd")
        self.run_motor(self.__motor3, right_speed, "fwd")
        self.run_motor(self.__motor4, right_speed, "fwd")

    def start_intake(self, speed:float=75, direction:str="fwd"):
        assert direction in ["fwd", "rev", "stop"]
        self.run_motor(self.__motor6, speed, direction)
        self.run_motor(self.__motor5, speed, direction)
    
    def stop_intake(self):
        self.run_motor(self.__motor6, 0, "stop")
        self.run_motor(self.__motor5, 0, "stop")

    def run_motor(self, motor, speed:float=0.0, motor_direction:str="fwd"):
        """
        Run a motor. Note that this sets the motor speed, and the motor keeps running once set, until set to speed 0
        """
        assert (speed >= -100) & (speed <= 100), "[ERR] Speed is out of bounds, must be a percent!"
        assert motor_direction in ['fwd','rev','stop'], "[ERR] Invalid Direction"
        if(motor_direction.lower().strip() == 'rev'):
            speed = speed *-1
        elif(motor_direction.lower().strip() == 'fwd'):
            speed = speed
        elif(motor_direction.lower().strip() == 'stop'): 
            speed = 0
        if(speed > 0):
            motor.forward(abs(speed)/100.0)
            if(motor.is_active):
                # print("[INFO] Motor Forward.")
                self.__rgbLED.color = (0,0,1)
        elif(speed <0):
            motor.backward(abs(speed)/100.0)
            if(motor.is_active):
                # print("[INFO] Motor Reverse.")
                self.__rgbLED.color = (1,0,0)   
        elif(speed==0):
            motor.stop()
            if(motor.is_active == False):
                self.__rgbLED.color = (0,0,0)
                if(self.__verbose==True):
                    print("[INFO] Motor Stopped.")
    
    def turn_continously(self, turn_dir:str="right", speed:float=100):
        """
        Turn robot continuously (tank/pivot turn)
        """
        turn_dir = turn_dir.lower().strip()
        assert (turn_dir == "right") or (turn_dir == "left"), "[ERR] Invalid Turn Direction Parameter."
        if (turn_dir == "right"):
            self.drive_motors(left_speed=speed, right_speed=-speed)
        elif(turn_dir == "left"):
            self.drive_motors(left_speed=-speed, right_speed=speed)
        else:
            print("[ERR] something went wrong")

    def stop_drive_motors(self):
        self.drive_motors(0,0)

    def stop_motors(self):
        self.stop_drive_motors()
        self.stop_intake()

    def run_avoidance_check(self, threshold, ignore = False):
        left_distance, right_distance = self.get_distances()
        print("check")
        if((left_distance<threshold) | (right_distance<threshold)):
            self.stop_drive_motors()
            time.sleep(5)

    def get_desired_heading(self):
        return self.__desired_heading
    
    #private member functions (the __ before the variable or function denotes it as private)
    def __heading_to_position(self, target_center):
        tgt_hdg = np.mod(np.degrees(np.arctan2(target_center[0]-self.__position[0],
                                               target_center[1]-self.__position[1]))+360,360)
        return tgt_hdg
    
    def get_button_state(self)->bool:
        return(self.__button.is_pressed)

    def get_on_state(self)->bool:
        return(self.__on_state)
    
    def get_distances(self):
        left_distance = 0
        right_distance = 0

        left_distance = self.__distance_sensor_left.distance * 100 #cm
        right_distance = self.__distance_sensor_right.distance * 100
        
        time.sleep(0.01)
        # if(self.__verbose==True):
        print(f"[DISTANCE SENSOR] Distance (CM): {left_distance} (LEFT), {right_distance} (RIGHT).", end="|")
        return(left_distance, right_distance)
        
    def __heading_to_angle(self, target_angles):
        #account for multiple targets? targets would be ping pong balls in this case
        if len(target_angles=0):
            #no targets detected to turn to, in this case, keep going at current heading
            return self.__heading
        
        relative_angle = 0
        angle_difference = 0
        for i in range(0, min(len(target_angles))):
            if angle_difference < abs(target_angles[i]):
                relative_angle = target_angles[i]
                angle_difference = abs(target_angles[i])

        tgt_hdg = self.__heading + relative_angle
        return tgt_hdg
    
    def get_timer(self):
        return self.__timer
    
    def get_competition_timer(self):
        return self.__competition_timer
    
    def __select_action(self):
        delta_angle = max(self.__desired_heading, self.__heading) - min(self.__desired_heading, self.__heading)
        
        # determine the angle between current and desired heading
        delta_angle = max(self.__desired_heading, self.__heading) - min(self.__desired_heading, self.__heading)

        if delta_angle > 0:
            if self.__heading > self.__desired_heading:
                pass
                #tank turn left until heading is correct
            elif self.__heading < self.__desired_heading:
                pass
                #tank turn right until heading is correct
            else:
                pass
                #drive straight as heading is correct

    def decide(self):
        while(True): #replace with while switch is on when switch enabled.
            if(self.__on_state):
                #TODO implement detect april tags, find angles to them?

                #TODO implement ping pong ball detection, find angles to them
                #TODO check if heading is correct, if not turn. else drive forward
                # self.__heading = #get heading from adcs system
                #check time, if time is running out use self.__heading_to_position(insert center of arena position here? whatever the final drop off is)
                print(f"The heading of the robot is {self.__heading}")
                # self.__desired_heading = self.__heading_to_angle(targets) #TODO implement targets (ping pong balls? fiducial/april tag)
                self.__select_action() #make this return a command?
                self.drive_fwd_continuosly(speed=100)
                # turn_continuously(turn_dir="clockwise",speed=100)
            elif(self.__on_state==False):
                autonomousController.stop_motors()

        pass

    def driveForTime(self, start_time:float=1, direction:str="forward", speed:float=100, duration:float=1):
        
        # print(f"dir{direction}", end="|")
        assert direction in ["forward", "reverse", "left", "right", "stop", "wall_left", "wall_forward"]
        if((direction=="forward") & (self.__timer >= start_time)):
            # if(self.__timer >= end_time - 0.2):
                # self.run_avoidance_check(50)
                # self.__timer -= 5
            self.drive_motors(speed, speed)
            print(f"dir{direction}", end="|")
        elif((direction=="reverse") & (self.__timer >= start_time)):
            print(f"dir{direction}", end="|")
            self.drive_motors(speed, speed)
        elif((direction=="left") & (self.__timer >= start_time)):
            self.drive_motors(0.05*speed, speed)
            print(f"dir{direction}", end="|")
            # self.drive_motors(5, speed)
        elif((direction=="right") & (self.__timer >= start_time)):
            self.drive_motors(speed, 0.05*speed)
            print(f"dir{direction}", end="|")
            # self.drive_motors(speed,5)
        elif((direction=="stop") & (self.__timer >= start_time)):
            self.drive_motors(0,0)
            print(f"dir{direction}", end="|")
            # self.drive_motors(speed,5)
        elif((direction=="wall_left") & (self.__timer >= start_time)):
            self.drive_motors(0.075*speed, speed)
            print(f"dir{direction}", end="|")
            # self.drive_motors(5, speed)
        elif((direction=="wall_forward") & (self.__timer >= start_time)):
            # autonomousController.switch_ultrasound_enable()
            self.drive_motors(0.5*speed, speed)
            print(f"dir{direction}", end="|")
            # self.drive_motors(5, speed)
        else:
            #something went wrong
            pass
        return (start_time + duration)
    
    def retrieve_percentage(self):
        self.__battery = psutil.sensors_battery()
        if(self.__battery != None):
            self.__percent = int(self.__battery.percent)
        else:
            self.__percent = -1
        return self.__percent
    
    def update(self):
        # assert self.__first_start == False, "[ERR] Must be run after button press"
        if(self.__first_start == False):
            self.__current_time = time.time()
            self.__timer = self.__current_time - self.__start_time
            self.__competition_timer = self.__current_time - self.__competition_start_time
        
        if(self.__camera_enabled):
            try:
                self.__image_processor.run()
            except:
                pass
        
        if(self.ultrasound_enabled==True):
            self.run_avoidance_check(10)
        elif(self.ultrasound_enabled==False):
            pass

            
        if(self.check_if_endgame(179)):
            #TODO have robot know to return to start
            autonomousController.stop_motors()
            sys.exit(0)
            pass
        
        if(self.__adcs_enabled):
            self.__adcs_system.update()
            self.__adcs_system.add_to_csv()
            # self.__raw_accel, self.__acceleration, self.__velocity, self.__position, self.__orientation = self.__adcs_system.get_data()
            # print(f"Raw:{(round(self.__raw_accel[1:][0],2), round(self.__raw_accel[1:][0],2),self.__raw_accel[1:][1])}|Accel:{self.__acceleration[1:]}|Vel:{self.__velocity[1:]}|Pos:{self.__position[1:]}|Rpy:{self.__orientation}")

        
        #Update and get sonar data, and check for collision
        self.get_distances()
        
       
        
        pass
if __name__ == "__main__":
    autonomousController = AutonomousController()
    
    while(True):
        # autonomousController.decide()
        automatic_start=True #change in comp
        motor_enable = True
        print() #new line
        print("l", end='>')
        if (autonomousController.retrieve_percentage() != -1):
            print(f"PWR{autonomousController.retrieve_percentage()}",end='|')
        print(f"on:{autonomousController.get_on_state()}", end='|')
        if(autonomousController.get_on_state() or automatic_start==True):
            autonomousController.update()
            print(f"timer:{autonomousController.get_timer()}-s", end='|')
            print(f"c_timer{autonomousController.get_competition_timer()}-s", end='|')
            if(motor_enable == True):
                # autonomousController.start_intake(100, direction="rev")
                # timestamp = autonomousController.driveForTime(0, "reverse", 100, 10)
                # sys.exit(0)
                timestamp = autonomousController.driveForTime(0, "wall_forward", 65, 1.75)
                # timestamp = autonomousController.driveForTime(timestamp, "wall_left", 100, 3.25)
                # timestamp = autonomousController.driveForTime(timestamp, "forward", 100, 0.75)
                # timestamp = autonomousController.driveForTime(timestamp, "right", 100, 3.75)
                # timestamp = autonomousController.driveForTime(timestamp, "forward", 100, 0.75)
                # timestamp = autonomousController.driveForTime(timestamp, "left", 100, 3.75)
                # timestamp = autonomousController.driveForTime(timestamp, "forward", 100,0.5 )
                # timestamp = autonomousController.driveForTime(timestamp, "left", 100, 0.5)
                # timestamp = autonomousController.driveForTime(timestamp, "forward", 100, 0.5)
                # timestamp = autonomousController.driveForTime(timestamp, "left", 100, 120-timestamp)
        else:
            autonomousController.stop_motors()
            
        time.sleep(0.1)
    # autonomousController.decide()