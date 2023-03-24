# import rpi.GPIO as GPIO
from gpiozero import LED
from time import sleep
from signal import pause
import sys

def gpio_test(pin=18):

    ledA = LED(pin)
    print("Setup")
    while True:
        print(f"testing/blinking pin {pin}")
        ledA.on()
        sleep(1)
        ledA.off()
        sleep(1)

if __name__ == "__main__":
    print(*sys.argv[1:])
    gpio_test(int(*sys.argv[1:]))
   

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
# print("PiSetup Finished!")
# while True:
    # GPIO.output(18, GPIO.HIGH)
    # print("on")
    # sleep(1)
    # print("off")
    # GPIO.output(18, GPIO.LOW)
    # sleep(1)