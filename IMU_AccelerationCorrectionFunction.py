import board
import math
import time
import adafruit_fxos8700
from IMU_Orientation_Functions import getPitch, getRoll

# i2c = board.I2C()
# sensor = adafruit_fxos8700.FXOS8700(i2c)

def getCorrectedAccel(i2c, sensor):
    """
    Corrects for gravity in the acceleration values.
    Args:
        sensor: the sensor being read
    Returns:
        List of acceleration values [x, y, z] without gravity in units of m/s^2
            getCorrectedAccel(sensor) -> [0, 0, 0]
    Raises:
        AssertionError: the sensor is not the accelerometer
    """

    assert type(sensor) is adafruit_fxos8700.FXOS8700
    accelX, accelY, accelZ = sensor.accelerometer

    pitch = getPitch(sensor)
    roll = getRoll(sensor)

    GRAVITY = 9.8

    #Correct for gravity in accelX
    gX = GRAVITY * math.sin(math.radians(pitch))
    corrected_accelX = accelX - gX

    #Correct for gravity in accelY
    gY = GRAVITY * math.sin(math.radians(roll))
    corrected_accelY = accelY - gY

    #Correct for gravity in accelZ
    gZtheta = math.atan(math.sqrt((math.tan(math.radians(roll))**2) + (math.tan(math.radians(pitch))**2)))
    gZ = GRAVITY * math.cos(gZtheta)
    if accelZ > 0:
        corrected_accelZ = accelZ - gZ
    else:
        corrected_accelZ = accelZ + gZ

    return corrected_accelX, corrected_accelY, corrected_accelZ
