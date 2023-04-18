from gpiozero import Servo

class ServoMotor(object):
    def __init__(self, pin:int=9, initial_degree:float=0.0):
        self.__pin = pin
        self.__degree = self.__initial_degree = initial_degree
        self.__value = self.__initial_value = self.__initial_degree/180.0

        self.__servoMotor = Servo(self.__pin, initial_value=self.__value)
        pass

    def rotateToDegree(self, deg:float=0):
        #constrain degree between -180 and 180 where 0 is the midpoint, scale this to -1 to 1 where 0 is the midpoint
        self.__degree = deg
        if(deg > 180):
            self.__servoMotor.max()
        if deg < 180:
            self.__servoMotor.min()
        if deg == 0:
            self.__servoMotor.mid()
        else:
            self.__value = self.__degree/180.0 
            self.__servoMotor.value = self.__value

    def resetToInitial(self):
        self.rotateToDegree(self.__initial_degree)

    
    




