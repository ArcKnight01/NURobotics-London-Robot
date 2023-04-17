#import imu data as a csv
#read a pandas dataframe from the csv
#should be accel x,y,z, and roll pitch yaw data
#linear interpolate (lerp) positions between values
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns

#get csv data from robot, and put it into a pandas dataframe
df = pd.read_csv("imu_data.csv")
