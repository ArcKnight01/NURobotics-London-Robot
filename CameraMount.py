from Servo_Motors import ServoMotor
import time
class CameraMount:
    def __init__(self, servo_pin_1, servo_pin_2):
        self.__top_servo = ServoMotor(servo_pin_1, 0)
        self.__bottom_servo = ServoMotor(servo_pin_2, 0)

    def moveToSphericalCoordinate(self, top_servo_deg, bottom_servo_deg):
        self.__top_servo.rotateToDegree(top_servo_deg)
        self.__bottom_servo.rotateToDegree(bottom_servo_deg)
        time.sleep(0.001)

if __name__ == '__main__':
    cameraMount = CameraMount(9,10)
    while(True):
        top_deg = int(input("top:"))
        bottom_deg = int(input("bottom:"))
        cameraMount.moveToSphericalCoordinate(90,90)