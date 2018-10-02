# Author: eunsunlee
# partial source: https://github.com/chanlhock/IMU/blob/master/imu.py
# integration of rk4: https://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf
# integration of rk4: https://github.com/scomup/imu_tools/blob/master/src/imu_tracker/common.h


# https://developer.android.com/reference/android/hardware/SensorManager.html#getRotationMatrix(float[],%20float[],%20float[],%20float[])
# https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/wiki/simple_algorithms
# https://github.com/matthew-brett/transforms3d/blob/master/original/transformations.py
# https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py


import socket, traceback
import csv
import struct
import sys, time, string, pygame
from pygame.locals import *
from ponycube import *
from madgwickahrs import *
import quaternion
from quaternion import QuaternionClass
from a3muse import quatNormalized, IntegrationRK4, AccMagOrientation, headingfromMag, QuatToEuler, angle_between, QuatToRotMat, AxisAngleToRotMat, RotMatToQuat
from math import atan2, atan
from numpy.linalg import inv
from numpy import linalg as LA



algorithm = "Madgwick"    #A3, MUSE, Madgwick, MUSE+A3

filename = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/imu_data.txt", "r")

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
Eg = 0

initial = 0
update = 0

Ax = []
Ay = []
Az = []

beta = 0.80

for i in filename: 
    my_array = i.split()
    length = len(my_array)
    if len(my_array) >= 13:
        ax = my_array[0]
        ay = my_array[1]
        az = my_array[2]
        gx = my_array[3]
        gy = my_array[4]
        gz = my_array[5]
        mx = my_array[6]
        my = my_array[7]
        mz = my_array[8]


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



############################ MUSE + A3 #####################################################
        if algorithm == "MUSE+A3":
            # # Initial yaw, pitch, roll from Accelerometer and Magnetometer 
            yawAM, pitchAM, rollAM, quatAM = AccMagOrientation(accel, mag)
            omega1 = [gx, gy, gz]
            dt = time_diff
            quatG = IntegrationRK4(omega0, omega1, quat, dt)
            yawG, pitchG, rollG = QuatToEuler(quatG)

            omega0 = omega1

            w = max(gyro)*180/math.pi
            a = max(accel)

            if w < 240 and a < 2*9.8:

                headingM = headingfromMag(mag)

                if initial < 50:
                    quat = quatAM
                    print("initial")
                    # O: orientation rotMat from quat 
                    # O-1 : inverse of the rot Mat 
                    # Calculate Ng = O*NL- Equation (1)
                    N_L = np.mat([[mx],[my],[mz]])
                    # print("N_L")
                    # print(N_L)
                    O = QuatToRotMat(quatAM)
                    N_G = O*N_L
                    # print("N_G")
                    # print(N_G)
                    initial = initial + 1

                    # O_hat = QuatToRotMat(quatG)  
                    # Oinv_hat = inv(O_hat)
                    # N_L_hat = Oinv_hat * N_G  
                    # print("N_L_hat")  
                    # print(N_L_hat)                
     


                else:

                    Eg = Eg + 0.0003*w*time_diff + 0.001*a*time_diff

                    if similaritywindow > 2:
                        # calculate pc and pg 
                        pc = 1/(2**np.var(np.subtract(Sc,C)))
                        pg = 1/(2**np.var(np.subtract(Sg,G)))

                        if pc > 0.2 and pg > 0.2: 
                            print("change?")



                        E1 = -32.14*pc + 19.93
                        E2 = -12.86*pg + 11.57
                        Ec = max(E1, E2)
                        Eg = Eg*1000

                        print(Ec)
                        print(Eg)
                        if Ec < Eg*1000: 

                            print("yes, update quat with AM")
                            quat = quatAM
                            update = 1

                        # Update 3D magnetic vector estimation
                        N_L = np.mat([[mx],[my],[mz]])
                        # print("N_L")
                        # print(N_L)
                        O = QuatToRotMat(quatAM)
                        N_G = O*N_L

                                                # reset values
                        similaritywindow = 0 
                        C = []
                        Sc = []
                        Sg = []
                        G = []
                        Eg = 0 


                    else:
                        C.append(headingM)
                        Sc.append(yawG)
                        Sg.append(rollG)
                        G.append(rollAM)
                        similaritywindow = similaritywindow + time_diff
                        
                    if update == 0:

                        O_hat = QuatToRotMat(quatG)  
                        Oinv_hat = inv(O_hat)
                        N_L_hat = Oinv_hat * N_G  
                        # print("N_L_hat")  
                        # print(N_L_hat)   

                        N_L = np.mat([[mx],[my],[mz]])
                        # print("N_L")
                        # print(N_L)

                        N_L_hat = np.array([np.array(N_L_hat)[0][0], np.array(N_L_hat)[1][0], np.array(N_L_hat)[2][0]])
                        N_L = np.array([mx, my, mz])
                        RotAxis = np.cross(N_L_hat, N_L)
                        RotAxis = RotAxis/LA.norm(RotAxis)
                        # print("RotAxis")
                        # print(RotAxis/LA.norm(RotAxis))
                        
                        alpha = 0.01
                        RotAngle = angle_between(N_L_hat, N_L)
                        alphaRotAngle = alpha* RotAngle
                        deltaRotMat = AxisAngleToRotMat(RotAxis, alphaRotAngle)
                        Onew_hat = np.array(np.mat(inv(deltaRotMat))*np.mat(O_hat))
                        quatMUSE = RotMatToQuat(Onew_hat)
                        quatMUSE = quatNormalized(quatMUSE)
                        quat = QuaternionClass(quatMUSE[0], quatMUSE[1], quatMUSE[2], quatMUSE[3])                

                        #print("update quat with MUSE")

                    update = 0

            qw = quat[0]
            qx = quat[1]
            qy = quat[3]
            qz = -quat[2]

############################ MUSE + A3 #####################################################



############################ MUSE #####################################################
        if algorithm == "MUSE":
            # # Initial yaw, pitch, roll from Accelerometer and Magnetometer 
            yawAM, pitchAM, rollAM, quatAM = AccMagOrientation(accel, mag)
            omega1 = [gx, gy, gz]
            dt = time_diff
            quatG = IntegrationRK4(omega0, omega1, quat, dt)
            yawG, pitchG, rollG = QuatToEuler(quatG)

            omega0 = omega1


            headingM = headingfromMag(mag)

            if initial < 50:
                quat = quatAM
                print("initial")
                    # O: orientation rotMat from quat 
                    # O-1 : inverse of the rot Mat 
                    # Calculate Ng = O*NL- Equation (1)
                N_L = np.mat([[mx],[my],[mz]])
                    # print("N_L")
                # print(N_L)
                O = QuatToRotMat(quatAM)
                N_G = O*N_L
                # print("N_G")
                # print(N_G)
                initial = initial + 1
            

            else:

                if similaritywindow > 2:
                    Ax = np.array(Ax)-9.8
                    Ay = np.array(Ax)-9.8
                    Az = np.array(Az)-9.8
                    
                    x_max = max([abs(max(Ax)), abs(min(Ax))])
                    y_max = max([abs(max(Ay)), abs(min(Ay))])
                    z_max = max([abs(max(Az)), abs(min(Az))])

                    xyz_min = min([x_max, y_max, z_max])
                    print(xyz_min)

                    # acceleration roughly measures 9.8m/s2

                    if xyz_min < 0.35: 
                        print("yes, update quat with AM")
                        Oa = QuatToRotMat(quatAM)
                        Og = QuatToRotMat(quatG)

                        Ocomp = np.mat(Oa)*(1-beta) + np.mat(Og)*beta
                        print("Oa")
                        print(Oa)
                        print("Og")
                        print(Og)
                        print("Ocomp")
                        print(Ocomp)
                        quatComp = RotMatToQuat(np.array(np.mat(Ocomp)))
                        quat = quatComp
                        update = 1

                        # Update 3D magnetic vector estimation
                    N_L = np.mat([[mx],[my],[mz]])
                        # print("N_L")
                        # print(N_L)
                    O = QuatToRotMat(quatAM)
                    N_G = O*N_L

                                                # reset values
                    similaritywindow = 0 
                    Ax = []
                    Ay = []
                    Az = []

                else:
                    Ax.append(ax)
                    Ay.append(ay)
                    Az.append(az)
                    similaritywindow = similaritywindow + time_diff
                        
                if update == 0:

                    O_hat = QuatToRotMat(quatG)  
                    Oinv_hat = inv(O_hat)
                    N_L_hat = Oinv_hat * N_G  
                    # print("N_L_hat")  
                    # print(N_L_hat)   

                    N_L = np.mat([[mx],[my],[mz]])
                     # print("N_L")
                    # print(N_L)

                    N_L_hat = np.array([np.array(N_L_hat)[0][0], np.array(N_L_hat)[1][0], np.array(N_L_hat)[2][0]])
                    N_L = np.array([mx, my, mz])
                    RotAxis = np.cross(N_L_hat, N_L)
                    RotAxis = RotAxis/LA.norm(RotAxis)
                    # print("RotAxis")
                    # print(RotAxis/LA.norm(RotAxis))
                        
                    alpha = 0.01
                    RotAngle = angle_between(N_L_hat, N_L)
                    alphaRotAngle = alpha* RotAngle
                    deltaRotMat = AxisAngleToRotMat(RotAxis, alphaRotAngle)
                    Onew_hat = np.array(np.mat(inv(deltaRotMat))*np.mat(O_hat))
                    quatMUSE = RotMatToQuat(Onew_hat)
                    quatMUSE = quatNormalized(quatMUSE)
                    quat = QuaternionClass(quatMUSE[0], quatMUSE[1], quatMUSE[2], quatMUSE[3])                

                    #print("update quat with MUSE")

                update = 0

            qw = quat[0]
            qx = quat[1]
            qy = quat[3]
            qz = -quat[2]

############################ MUSE #####################################################




############## A3 #####################################################################

        if algorithm == "A3":
            # A3 Algorithm - gyroscope calibration
            omega1 = [gx, gy, gz]
            dt = time_diff
            quatG = IntegrationRK4(omega0, omega1, quat, dt)
            yawG, pitchG, rollG = QuatToEuler(quatG)
            omega0 = omega1
            # A3 Algorithm - accelerometer, magnetometer calibration
            yawAM, pitchAM, rollAM, quatAM = AccMagOrientation(accel, mag)
            # TODO: Update quaternion if w < 240 degree, a < 2g 
            w = max(gyro)*180/math.pi
            a = max(accel)

            if w < 240 and a < 2*9.8:

                print(w)
                print(a)
                headingM = headingfromMag(mag)
                if similaritywindow > 2:
                    # print("similaritywindow")
                    # calculate pc and pg 
                    pc = 1/(2**np.var(np.subtract(Sc,C)))
                    pg = 1/(2**np.var(np.subtract(Sg,G)))
                    print(pc)
                    print(pg)
                    if pc > 0.2 and pg > 0.2: 
                        print("change?")
                        # TODO: if Ec < Eg, then update quaternion
                        E1 = -32.14*pc + 19.93
                        E2 = -12.86*pg + 11.57
                        Ec = max(E1, E2)
                        Eg = (Eg + 0.0003*w*time_diff + 0.001*a*time_diff)*1000

                        print(Ec)
                        print(Eg)
                        if Ec < Eg*1000: 

                            print("yes")
                            quat = quatAM
                #         quat = quatAM

                        else:
                            quat = quatG
                    # reset values
                    similaritywindow = 0 
                    C = []
                    Sc = []
                    Sg = []
                    G = []
                    Eg = 0 
                else:
                #     #TODO: update Eg 
                    Eg = Eg + 0.0003*w*time_diff + 0.001*a*time_diff
                    C.append(headingM)
                    Sc.append(yawG)
                    Sg.append(rollG)
                    G.append(rollAM)
                    similaritywindow = similaritywindow + time_diff
                    quat = quatG

                # if initial <50:
                #     quat = quatAM
                #     print("initial")
                #     initial = initial + 1

                # else:
                #     print("not initial")
                #     quat = quatG


                qw = quat[0]
                qx = quat[1]
                qy = quat[3]
                qz = -quat[2]





############## A3 #####################################################################


############## Madgwick Algorithm #####################################################

        if algorithm == "Madgwick":
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


############## Madgwick Algorithm #####################################################



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




