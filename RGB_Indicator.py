
from gpiozero import RGBLED

class RGB_Indicator(object):
    def __init__(self, enable, verbose, red_pin:int=23, green_pin:int=24, blue_pin:int=25, pwm:bool=True, active_high:bool=True, initial_color:tuple=(255,0,0)):
        self.__color = initial_color
        self.__unit_color = map(self.rgb_to_unit, self.__color)
        self.__rgbLED = RGBLED(red_pin,green_pin,blue_pin, active_high=True, pwm=True, initial_value=self.__unit_color)

    def __repr__(self):
        return(f"")

    def set_color(self, r:int=None, g:int=None, b:int=None, color:tuple=None, default_color:tuple=(255,0,0)):
        assert ((r and g and b) or (color)) and not ((r or g or b) and (color)), ValueError
        #depending on whether r,g,b values are passed individually or as a tuple, set the
        if(r and g and b):
            self.__color = (r,g,b)
        elif(color):
            self.__color = color
        else:
            self.__color = default_color
        
        #convert the color in rgb format to unit format 
        self.__unit_color = map(self.rgb_to_unit,self.__color)
        self.__rgbLED.color = self.__unit_color

    def rgb_to_unit(self, val):
                return val/255
    
    