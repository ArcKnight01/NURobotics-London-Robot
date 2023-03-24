import sys
import pathlib
import datetime

import time
import cv2
import numpy as np
import os


import picamera
print(f"{os.uname()}")
from Camera_Util import detect_apriltags
from Camera_Util import detect_spheres

class ImageProcessor():
    def __init__(self, log_dir = './'):
        self.__camera = picamera.PiCamera()
        self.__camera.resolution = (640, 480)
        self.__camera.framerate = 24
        time.sleep(2) #camera warm up time
        self.__image = np.empty((480*640*3,), dtype=np.uint8)

        #create image save directory
        self.__image_dir = pathlib.Path(log_dir,'Frames')
        if(self.__image_dir.exists() == False):
            print(f"{self.__image_dir} does not exist, creating directory.")
        self.__image_dir.mkdir(parents=True, exist_ok=True)
    
    def run(self):
        try:
            self.__camera.start_preview()
            self.__camera.capture(self.__image, 'bgr')
        except:
            # restart the camera
            # self.__camera = picamera.PiCamera()
            self.__camera.resolution = (640, 480)
            self.__camera.framerate = 24
            time.sleep(2) # camera warmup time
            
        image = self.__image.reshape((480, 640, 3))
        
        #detect APRIL TAGS
        detected, image, tagFamilies, tagIds, centers, angles, corners = detect_apriltags(image)
        if(detected == True):
            print(f"TAG(s) DETECTED!")
        else:
            print(f"NO TAG(s) DETECTED!")
        #detect SPHERES
        # detect_spheres(image)

        # log the image
        fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
        print(f"Took image {fn}.")
        cv2.imwrite(str(fn), image)

if __name__ == '__main__':
    imageprocessor = ImageProcessor(log_dir='/home/pi/NuRobotics/')
    imageprocessor.run()