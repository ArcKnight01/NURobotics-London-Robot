import cv2
import matplotlib.pyplot as plt
from matplotlib import cm
import sys
from numpy.core.numeric import ones
from time import sleep
import apriltag
import numpy as np
def sensor_position(pix_x, pix_y, res_x, res_y):
    sensor_width,sensor_height = (0.00368, 0.00276) #mm to meters
    origin = (res_x/2,res_y/2)
    ratio_x, ratio_y = (sensor_width/res_x, sensor_height/res_y)
    
    pix_x, pix_y = (pix_x - origin[0], pix_y - origin[1])

    sensor_pos_x, sensor_pos_y = (ratio_x*pix_x, ratio_y*pix_y)
    
    return (sensor_pos_x, sensor_pos_y)


focal_length = 0.00304 #mm is the focal length

def sensor_angle(sensor_pos_x, sensor_pos_y, f):
    return np.degrees(np.arctan2(sensor_pos_x,f))

def detect_apriltags(image):
    # image = cv2.imread('/home/pi/NuRobotics/Frames/tag16_05_00000.png')
    
    #convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    options = apriltag.DetectorOptions(families=["tag36h11","tag16h5"])
    detector = apriltag.Detector(options)
    
    results = detector.detect(gray)
    centers = []
    angles = []
    corners = []
    tagFamilies = []
    tagIds = []
    tag_detected = False
    for r in results:
        #find the corners of the april tag
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        
        corners.append((ptA, ptB, ptC, ptD))
        #find the centers
        center = (int(r.center[0]), int(r.center[1]))
        center.append(center)
        
        #find the angles 
        a = sensor_position(center[0], center[1], image.shape[1], image.shape[0])
        angle = sensor_angle(a[0], a[1], focal_length)
        angles.append(angle)

        print(f"[INFO] tag center @ {center}")
        print(f"[INFO] tag angle @ {angle}")
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

       
        # draw the center (x, y)-coordinates of the AprilTag
        cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), -1)
        
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        tagId = r.tag_id.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        print(f"[INFO] tag family: {tagFamily}")
        tagFamilies.append(tagFamily)
        print(f"[INFO tag id {tagId}]")
        tagIds.append(tagId)
    
    tag_detected = True if len(results) != 0 else False
    return tag_detected, image, tagFamilies, tagIds, centers, angles, corners
    
