
from gpiozero import Motor
from gpiozero import RGBLED
from gpiozero import Button
from gpiozero import DistanceSensor
from ADCS_System import *
from ImageProcessing import *
class AutonomousController(object):
    def __init__(self,
                robot_state,
                motor1_pins=(15,14,18), 
                motor2_pins=(7,8,12), 
                motor3_pins=(5,6,13), 
                motor4_pins=(26,20,19), 
                motor5_pins=(9,11,10), 
                motor6_pins=(27,17,22),
                rgb_pins = (23,24,25),
                button_pin = 4,
                distance_sensor_left_pin = (0,1),
                distance_sensor_right_pin = (16,21)

                ):
        self.__motor1 = Motor(forward= motor1_pins[0], backward= motor1_pins[1], enable= motor1_pins[2], pwm=True) #left front
        self.__motor2 = Motor(forward= motor2_pins[0], backward= motor2_pins[1], enable= motor2_pins[2], pwm=True) #left back
        self.__motor3 = Motor(forward= motor3_pins[0], backward= motor3_pins[1], enable= motor3_pins[2], pwm=True) #right front
        self.__motor4 = Motor(forward= motor4_pins[0], backward= motor4_pins[1], enable= motor4_pins[2], pwm=True) #right back
        self.__motor5 = Motor(forward= motor5_pins[0], backward= motor5_pins[1], enable = motor5_pins[2], pwm=False) #intake left
        self.__motor6 = Motor(forward= motor6_pins[0], backward= motor6_pins[1], enable= motor6_pins[2], pwm=False) #intake right
        
        self.__rgbLED = RGBLED(rgb_pins[0],rgb_pins[1],rgb_pins[2], active_high=True, pwm=True, initial_value=(1,0,0))
        self.__button = Button(button_pin)
        self.__distance_sensor_left = DistanceSensor(echo=distance_sensor_left_pin[0], trigger=distance_sensor_left_pin[1])
        self.__distance_sensor_right = DistanceSensor(echo=distance_sensor_right_pin[0], trigger=distance_sensor_right_pin[1])

        self.__verbose = False
        self.__heading = None
        self.__desired_heading = None
        self.__on_state = False
        self.__button.when_pressed = switchOnState
        # self.__speed = None
        # self.__position = None

        pass
    
    def initialize(self, robot_state):
        #initialize heading
        self.__heading = robot_state['heading']
        self.__speed = robot_state['speed']
        self.__position = robot_state['position']
        self.__desired_heading = robot_state['heading']

    def __repr__(self):
        pass
    def update():
        pass
        
    def drive_motors(self, left_speed:float=0.0, right_speed:float=0.0):
        self.run_motor(self.__motor1, left_speed, "fwd")
        self.run_motor(self.__motor2, left_speed, "fwd")
        self.run_motor(self.__motor3, right_speed, "fwd")
        self.run_motor(self.__motor4, right_speed, "fwd")

    def run_intake(self, speed):
        # self.run_motor(self.__motor6, speed, "fwd")
        self.run_motor(self.__motor5, speed, "fwd")

    def run_motor(self, motor, speed:float=0.0, motor_direction:str="fwd"):
        assert (speed >= -100) & (speed <= 100), "ERR: Motor Speed Out of Bounds."
        if(motor_direction.lower().strip() == 'rev'):
            speed = speed *-1
        elif(motor_direction.lower().strip() == 'fwd'):
            speed = speed
        else: 
            speed = 0
        assert (speed >= -100) & (speed <= 100), "[ERROR] Speed is out of bounds, must be a percent!"
        if(speed > 0):
            motor.forward(abs(speed)/100.0)
            if(motor.is_active):
                print("[INFO] Motor Forward.")
                self.__rgbLED.color = (0,0,1)
        elif(speed <0):
            motor.backward(abs(speed)/100.0)
            if(motor.is_active):
                print("[INFO] Motor Reverse.")
                self.__rgbLED.color = (1,0,0)   
        else:
            motor.stop()
            if(motor.is_active == False):
                self.__rgbLED.color = (0,0,0)
                print("[INFO] Motor Stopped.")
    def drive_fwd_continuosly(self, speed):
        drive_motors(left_speed=speed,right_speed=speed)

    def turn_continously(self, turn_dir:str="clockwise", speed:float=100):
        """
        Turn robot continuously (tank/pivot turn)
        """
        turn_dir = turn_dir.lower().strip()
        assert (turn_dir == "clockwise") or (turn_dir == "counterclockwise"), "[ERR] Invalid Turn Direction Parameter."
        if (turn_dir == "clockwise"):
            self.drive_motors(left_speed=speed, right_speed=-speed)
        elif(turn_dir == "counterclockwise"):
            self.drive_motors(left_speed=-speed, right_speed=speed)
        else:
            print("ERR: something went wrong")

    def stop_drive_motors(self):
        self.drive_motors(0.0,0.0)
    
    def stop_intake(self):
        self.runIntake(0)

    def stop_motors(self)
        self.stop_drive_motors
        self.stop_intake

    def run_avoidance_check(self, threshold):
        left_distance, right_distance = getSonarDistances()
        if(left_distance>=threshold & right_distance>=threshold):
            #do something?
            return(True,True)
        elif(left_distance>=threshold & right_distance<threshold):
            #do something?
            return(True,False)
        elif(left_distance<threshold & right_distance>=threshold):
            #do something?
            return(False,True)
        elif(left_distance<threshold & right_distance<threshold):
            stop.drive_motors()
            return(False,False)

    def get_desired_heading(self):
        return self.__desired_heading
    
    #private member functions (the __ before the variable or function denotes it as private)
    def __heading_to_position(self, target_center):
        tgt_hdg = np.mod(np.degrees(np.arctan2(target_center[0]-self.__position[0],
                                               target_center[1]-self.__position[1]))+360,360)
        return tgt_hdg
    
    def getButtonState()->bool:
        return(self.__button.is_pressed)

    def getOnState()->bool:
        return(self.__on_state)
    
    def getSonarDistances():
        left_distance = self.__distance_sensor_left.distance * 100 #cm
        right_distance = self.__distance_sensor_right.distance * 100
        if(self.__verbose):
            print(f"[DISTANCE SENSOR] Distance (CM): {left_distance} (LEFT), {right_distance} (RIGHT).")
        return(left_distance, right_distance)
        
    def switchOnState():
        if self.__on_state == True:
            self.__on_state = False
        elif self.__on_state == False:
            self.__on_state = True
    
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
                self.__desired_heading = self.__heading_to_angle(targets) #TODO implement targets (ping pong balls? fiducial/april tag)
                self.__select_action() #make this return a command?
                drive_fwd_continuosly(speed=100)
                # turn_continuously(turn_dir="clockwise",speed=100)
            else(self.__on_state==False):
                autonomousController.stop

        pass
if __name__ == "__main__":
    autonomousController = AutonomousController()
    
    while(True):
        if(autonomousController.getOnState):
            # autonomousController.drive_motors(100,100)
            autonomousController.update()
            autonomousController.run_intake(100)
            autonomousController.turn_continously("counterclockwise", 50)
        else:
            autonomousController.stop_motors
    # autonomousController.decide()