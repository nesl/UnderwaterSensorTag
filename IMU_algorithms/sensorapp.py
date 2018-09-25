# Author: eunsunlee
# partial source: https://github.com/chanlhock/IMU/blob/master/imu.py
# integration of rk4: https://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf
# integration of rk4: https://github.com/scomup/imu_tools/blob/master/src/imu_tracker/common.h

import socket, traceback
import csv
import struct
import sys, time, string, pygame
from pygame.locals import *
from ponycube import *
from madgwickahrs import *
import quaternion
from quaternion import QuaternionClass
from a3 import IntegrationRK4,computeOmegaskew, QuatToRotMat, RotMatToQuat


accel = [[0.0190, -0.0522, -0.9780],
         [0.0269, -0.0327, 0.9897],
         [0,-0.0093,1.0205],
         [0.0112, -0.0327, 1.0015],
         [0.0151, -0.0327, 1.0015]]

gyro = [[-0.9375, -1.25, 0.875],
        [-1.3125, -2, 0.125],
        [-1.5625, -2.3125, -0.1875],
        [-1, -1.9375, -0.0625],
        [-0.6250, -1.3125, 0]]
mag = [[0.20996090, 0.03125, -0.4487305],
       [0.2148438, 0.04101563, -0.4536133],
       [0.2148438, 0.04101563, -0.4536133],
       [0.2148438, 0.04101563, -0.4536133],
       [0.2148438, 0.04101563, -0.4536133]]





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

# a3quat = QuaternionClass(1, 0, 0, 0)

dt = 1/256

while 1:
    for i in range(len(mag)):
        ax = accel[i][0]
        ay = accel[i][1]
        az = accel[i][2]
        gx = gyro[i][0]
        gy = gyro[i][1]
        gz = gyro[i][2]
        mx = mag[i][0]
        my = mag[i][1]
        mz = mag[i][2]


        # A3 Algorithm gyroscope calibration
        omega1 = [gx, gy, gz]
        quat = IntegrationRK4(omega0, omega1, quat, dt)
        omega0 = omega1


        # Madgwick Algorithm
        # Imupredict = MadgwickAHRS();
        # Imupredict.quaternion = quat
        # Imupredict.sampleperiod = dt
        # Imupredict.update(gyro,accel,mag)
        # quat = Imupredict.quaternion

        qw = quat[0]
        qx = quat[1]
        qy = quat[3]
        qz = -quat[2]

        q.w = qw
        q.x = qx
        q.y = qy
        q.z = qz
        q = q.normalized()
            
        print("quat")
        print(q.w, q.x, q.y, q.z)


        cube.erase(screen)
        cube.draw(screen,q)
        pygame.display.flip()
        pygame.time.delay(0)
        event = pygame.event.poll()
    if event.type == pygame.QUIT \
        or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
        break
    if i == len(mag):
        i = 0








