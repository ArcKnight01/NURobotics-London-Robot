from gpiozero import Motor
from RGB_Indicator import RGB_Indicator

class DCMotor(object):
    def __init__(self, verbose:bool=False, enabled:bool=True, pins:list=(15,14,18), pwm:bool=True, rgbLED:RGB_Indicator=RGB_Indicator(enable=True, verbose=True, red_pin=23, green_pin=24, blue_pin=25)):
        self.__motor = Motor(forward= pins[0], backward= pins[1], enable= pins[2], pwm=True) #left front
        # self.__rgbLED = rgbLED
        self.__verbose = verbose
        self.__enabled = enabled
        pass

    def run(self, speed:float=0.0, motor_direction:str="stop"):
        """
        Run a motor. Note that this sets the motor speed, and the motor keeps running once set, until set to speed 0
        """
        if (self.__enabled):
            assert (speed >= -100) & (speed <= 100), "[ERR] Speed is out of bounds, must be a percent!"
            assert motor_direction in ['fwd','rev','stop'], "[ERR] Invalid Direction"
            if(motor_direction.lower().strip() == 'rev'):
                speed = speed *-1
            elif(motor_direction.lower().strip() == 'fwd'):
                speed = speed
            elif(motor_direction.lower().strip() == 'stop'): 
                speed = 0
            
            if(speed > 0):
                self.__motor.forward(abs(speed)/100.0)
                if(self.__motor.is_active):
                    print("[INFO] Motor Forward.")
                    # self.__rgbLED.set_color(0,0,255)
            elif(speed <0):
                self.__motor.backward(abs(speed)/100.0)
                if(self.__motor.is_active):
                    print("[INFO] Motor Reverse.")
                    # self.__rgbLED.set_color(color=(255,0,0))  
            elif(speed==0):
                self.__motor.stop()
                if(self.__motor.is_active == False):
                    # self.__rgbLED.set_color(0,0,0)
                    if(self.__verbose==True):
                        print("[INFO] Motor Stopped.")

# class IntakeMotor(DCMotor):
#     def __init__(self, verbose: bool = False, enabled:bool=False, pins: list = (15, 14, 18), pwm: bool = True, rgbLED: RGB_Indicator = RGB_Indicator(enable=True, verbose=True, red_pin=23, green_pin=24, blue_pin=25)):
#         super().__init__(verbose=verbose, enabled=enabled, pins=pins, pwm=pwm, rgbLED=rgbLED)


class DriveMotors(object):
    def __init__(self, 
                leftFrontMotor:DCMotor,
                rightFrontMotor:DCMotor,
                leftBackMotor:DCMotor,
                rightBackMotor:DCMotor
                ):
        
        self.__leftFrontMotor = leftFrontMotor
        self.__rightFrontMotor = rightFrontMotor
        self.__leftBackMotor = leftBackMotor
        self.__rightBackMotor = rightBackMotor
        pass

    def drive_motors(self, left_speed:float=0.0, right_speed:float=0.0):
        self.__leftFrontMotor.run(left_speed, "fwd")
        self.__rightFrontMotor.run(left_speed, "fwd")
        self.__leftBackMotor.run(right_speed, "fwd")
        self.__rightBackMotor.run(right_speed, "fwd")

    def stop_drive_motors(self):
        self.drive_motors(0,0)

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