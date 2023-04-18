#import imu data as a csv
#read a pandas dataframe from the csv
#should be accel x,y,z, and roll pitch yaw data
#linear interpolate (lerp) positions between values
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns

from sklearn.linear_model import LinearRegression


#get csv data from robot, and put it into a pandas dataframe
def main():
    #read the csv data into a pandas dataframe
    df = pd.read_csv("./imu_data.csv", sep =',', delimiter=',', header=['Time','AccelX', 'AccelY','AccelZ', 'VelX', 'VelY', 'VelZ', 'PosX', 'PosY', 'PosZ', 'Roll', 'Pitch', 'Yaw'])
    print(df.columns)
    # #Drop the accelZ and velocityZ data because won't be used in my analysis
    # df.drop(['AccelZ','VelZ'], axis=1, inplace=True)
    
    plt.plot(df["Time"], df["AccelX"], c='red', label='X')
    plt.plot(df["Time"], df["AccelY"],c='green', label='Y')
    plt.plot(df["Time"], df["AccelZ"],c='blue', label='Z')

    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('Robot\'s Acceleration Data over Time')
    plt.grid()
    plt.legend()
    plt.show()
    
    plt.plot(df["Time"], df["VelX"], c='red', label='X')
    plt.plot(df["Time"], df["VelY"], c='green', label='Y')
    plt.plot(df["Time"], df["VelZ"], c='blue', label='Z')


    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Robot\'s Velocity Data over Time')
    plt.grid()
    plt.legend()
    plt.show()

    plt.plot(df["Time"], df["PosX"],c='red', label='X')
    plt.plot(df["Time"], df["PosY"], c='green', label='Y')
    plt.plot(df["Time"], df["PosZ"], c='blue', label='Z')


    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Robot\'s Position Data over Time')
    plt.grid()
    plt.legend()
    plt.show()

    plt.plot(df["Time"], df["Roll"], c='red', label='Roll')
    plt.plot(df["Time"], df["Pitch"], c='green', label = 'Pitch')
    plt.plot(df["Time"], df["Yaw"], c='blue', label='Yaw')

    plt.xlabel('Time (s)')
    plt.ylabel('Degrees')
    plt.title('Robot\'s Roll-Pitch-Yaw(RPY) Data over Time')
    plt.grid()
    plt.legend()
    plt.show()



    #plot position data on a 3D plot
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_facecolor(color=(0.5,0.5,0.5))
    x = df["PosX"]
    y = df["PosY"]
    z = df["PosZ"]

    ax.scatter(x, y, z, marker='o', c='r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    
    ax.set_title("3D plot of robot's position data")
    plt.tight_layout()
    plt.show()




if __name__ == '__main__':
    main()
    pass
