from Sensor import Sensor
from gpiozero import DistanceSensor
import time

class Sonar(Sensor):
    def __init__(self, verbose=False, enable=True, echo_pin=0, trig_pin=1):
        super().__init__(verbose=verbose, enable=enable)
        self.__enable = super().__enable
        self.__verbose = super().__verbose
        if (self.__enable):
            self.__sensor = DistanceSensor(echo=echo_pin, trig=trig_pin)
            self.__sensor.max_distance = 100*100 #cm
            self.__distance = 0
    
    def update(self):
        if(self.__enable):
            self.__distance = self.__sensor.distance * 100 #cm
            time.sleep(0.01)
            
    def get_distance(self):
        self.update()
        if(self.__verbose):
                print(f"[SONAR SENSOR] Distance: {self.__distance} cm.", end="|")  
        return(self.__distance)
    
    def avoidance_check(self, threshold):
        distance = self.get_distance()
        if (distance < threshold):
            if(self.__verbose):
                print(f"[SONAR SENSOR] Obstacle detected ({self.__distance} cm away)")
            return(True)
        elif (distance  > threshold):
            if(self.__verbose):
                print(f"[SONAR SENSOR] No obstacle detected ({self.__distance} cm away)")
            return(False)
    

    

        
        
