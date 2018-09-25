# Author: eunsunlee
# partial source: https://github.com/chanlhock/IMU/blob/master/imu.py
# integration of rk4: https://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf
# integration of rk4: https://github.com/scomup/imu_tools/blob/master/src/imu_tracker/common.h


# https://developer.android.com/reference/android/hardware/SensorManager.html#getRotationMatrix(float[],%20float[],%20float[],%20float[])
# https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/wiki/simple_algorithms


import socket, traceback
import csv
import struct
import sys, time, string, pygame
from pygame.locals import *
from ponycube import *
from madgwickahrs import *
import quaternion
from quaternion import QuaternionClass
from a3 import IntegrationRK4, AccMagOrientation, headingfromMag, QuatToEuler
from math import atan2, atan



host = ''
port = 5555

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))


pygame.init()
screen = Screen(480,400,scale=1.5)
cube = Cube(40,30,60)
q = Quaternion(1,0,0,0)
incr = Quaternion(0.96,0.01,0.01,0).normalized()
cube.erase(screen)
cube.draw(screen,q)

previous_timestamp = 0

quat = QuaternionClass(1, 0, 0, 0)
omega0 = [0,0,0]

similaritywindow = 0
Sc = []
Sg = []
C = []
G = []

while 1:
    message, address = s.recvfrom(8192)
#    print(message)
    my_array = message.split(b',')
    length = len(my_array)
    if len(my_array) >= 13:
        ax = float(my_array[2])
        ay = float(my_array[3])
        az = float(my_array[4])
        gx = float(my_array[6])
        gy = float(my_array[7])
        gz = float(my_array[8])
        mx = float(my_array[10])
        my = float(my_array[11])
        mz = float(my_array[12])

        if previous_timestamp <= 0:
            previous_timestamp = float(my_array[0])
            time_diff = float(0)
        else:
            time_diff = float(my_array[0]) - previous_timestamp
            previous_timestamp = float(my_array[0])
                
        # print('Time taken:', time_diff, 'secs')

        accel = [ax, ay, az]
        gyro = [gx, gy, gz]
        mag = [mx, my, mz]
        # print(gyro)


        # # A3 Algorithm - gyroscope calibration
        # omega1 = [gx, gy, gz]
        # dt = time_diff
        # quatG = IntegrationRK4(omega0, omega1, quat, dt)
        # yawG, pitchG, rollG = QuatToEuler(quatG)
        # omega0 = omega1

        # # A3 Algorithm - accelerometer, magnetometer calibration
        # yawAM, pitchAM, rollAM, quatAM = AccMagOrientation(accel, mag)
    
        

        # headingM = headingfromMag(mag)

        # if similaritywindow > 2:
        #     # calculate pc and pg 
        #     pc = 1/(2**np.var(np.subtract(Sc,C)))
        #     pg = 1/(2**np.var(np.subtract(Sg,G)))
            
        #     # E1 = -32.14*pc + 19.93
        #     # E2 = -12.86*pg + 11.57

        #     # Ec = max(E1, E2)

        #     if pc > 0.2 and pg > 0.2: 
        #         quat = quatAM
        #         print("change!")
        #     else:
        #         quat = quatG
        #     # reset values
        #     similaritywindow = 0 
        #     C = []
        #     Sc = []
        #     Sg = []
        #     G = []
        # else:
        #     C.append(headingM)
        #     Sc.append(yawG)
        #     Sg.append(rollG)
        #     G.append(rollAM)
        #     similaritywindow = similaritywindow + time_diff
        #     quat = quatG

        #Madgwick Algorithm
        Imupredict = MadgwickAHRS();
        Imupredict.quaternion = quat
        Imupredict.sampleperiod = time_diff
        Imupredict.update(gyro,accel,mag)
        quat = Imupredict.quaternion

        qw = quat[0]
        qx = quat[1]
        qy = quat[3]
        qz = -quat[2]

        q.w = qw
        q.x = qx
        q.y = qy
        q.z = qz

        q = q.normalized()
        
        
#         print("quat from phone")
#         print(q.w, q.x, q.y, q.z)
# #        print("quat from pony")

#         print("result")
#         print(q.w, q.x, q.y, q.z)
        cube.erase(screen)
        cube.draw(screen,q)
        pygame.display.flip()
        pygame.time.delay(0)
        event = pygame.event.poll()
        if event.type == pygame.QUIT \
            or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            break




