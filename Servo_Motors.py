from gpiozero import Servo

class ServoMotor(object):
    def __init__(self, pin:int, initial_speed:float=0.0):
        self.__pin = pin
        self.__initial_speed = initial_speed
        self.__speed = self.__initial_speed
        self.__servoMotor = Servo(self.__pin, initial_value=0)
        
        pass

    def rotateToDegree(self, deg:float):
        #constrain degree between -180 and 180 where 0 is the midpoint, scale this to -1 to 1 where 0 is the midpoint
        if(deg > 180):
            self.__servoMotor.max()
        if deg < 180:
            self.__servoMotor.min()
        if deg == 0:
            self.__servoMotor.mid()
        else:
            val = deg/180.0 
            self.__servoMotor.value = val
    




