from gpiozero import LED
import time
from time import sleep
from signal import pause
from gpiozero import Motor
from gpiozero import Servo
from gpiozero import RGBLED
from colorzero import Color
from picamera import PiCamera
import numpy as np
import sys
import os
import board
import busio
import adafruit_bno055


#motor pins
ain1, ain2, ena = (5,6,19)
bin1, bin2, enb = (23,24,18)

#rgb led pins
rgb_r, rgb_g, rgb_b = (16, 20, 21)

#i2c sensors
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

#camera
# camera = PiCamera()

#motors
motor1 = Motor(ain1,ain2,enable=ena, pwm=True)
motor2 = Motor(bin1,bin2,enable=enb, pwm=True)

#rgb led
rgbled = RGBLED(rgb_r,rgb_b,rgb_g)
rgbled.color = (0,0,1)

#print outs
print(f"Motor1: AIN1:{ain1}, AIN2:{ain2},ENA:{ena}")
print(f"Motor2: BIN1:{bin1}, BIN2:{bin2},ENB:{enb}")

print("Setup Finished!")


def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

while True:
    
    print("Motor Loop")
    motor1.backward(1)
    # motor2.forward(0.5)
    
    print(f"Acclerometer {sensor.acceleration}")
    print(f"Magnetometer {sensor.magnetic}")
    print(f"Gyroscope {sensor.gyro}")
    print(f"Euler Angle {sensor.euler}")
    print(f"Quaternion {sensor.quaternion}")
    print(f"Linear Acceleration {sensor.linear_acceleration}")
    print(f"Gravity {sensor.gravity}")
    print()

    
    
    sleep(1)


  
    
# def capture():
#     print(f"taking a picture...")
#     t = time.strftime("_%H%M%S")      # current time string
#     imgname = ('/home/pi/NuRobotics/images/image_%s%s%s.jpg' % (name,t,target_angle)) #change directory to your fold>
#     print(imgname)
#     camera.start_preview()
    
#     time.sleep(0.05)
#     camera.capture(imgname)
#     pass


# if __name__ == "__main__":
    # print(*sys.argv[1:])
