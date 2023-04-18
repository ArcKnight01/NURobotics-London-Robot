from gpiozero import LED
import time
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
import cv2
import apriltag
import argparse
import pathlib

camera = PiCamera()
led = LED(12)
print("Setup Finished!")

def capture(filename):
    print(f"taking a picture...")
    t = time.strftime("_%H%M%S")      # current time string
    imgname = ('/home/pi/NuRobotics/images/image_%s_%s.jpg' % (filename,t)) #change directory to your fold>
    print(imgname)
    camera.start_preview()
    
    time.sleep(0.05)
    camera.capture(imgname)
    detect_apriltags(imgname)
    pass


def detect_apriltags(filename):
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-i", "--image", required=True, help="path to input containing AprilTag")
    # args = vars(ap.parse_args())
    image = cv2.imread('/home/pi/NuRobotics/Frames/tag16_05_00000.png')
    
    # image = cv2.imread(filename)
    # cv2.imwrite(f"{filename}-saved.jpg", image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    for r in results:
        print(f"tag {r.tag_id}detected!")
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(tagFamily))
    
    # show the output image after AprilTag detection
    # cv2.imshow("Image", image)
    cv2.imwrite(f"{filename}-saved-with-bounds.jpg", image)
    # cv2.waitKey(0)
    pass
if __name__ == "__main__":
    capture(*sys.argv[1:])