# author : eunsunlee
# This file reads values from imu and optitrack file save them into new formatted files



from modules.mpulib import computeheading, attitudefromCompassGravity, RP_calculate, MadgwickQuaternionUpdate, Euler2Quat, quaternion_to_euler_angle, MPU9250_computeEuler
import socket, traceback
import csv
import struct
import sys, time, string, pygame

import pygame
import pygame.draw
import pygame.time
import numpy as np
from math import sin, cos, acos
from modules.euclid import Vector3, Quaternion
from modules.EuclidObjects import Cube, Screen, Grid, PerspectiveScreen

import math

# from pygame.locals import *
# from ponycube import *
from modules.madgwickahrs import *
import modules.quaternion
from modules.quaternion import QuaternionClass
from modules.a3muse import androidAccMag2Euler, qnormalized, quatNormalized, IntegrationRK4, EulerToQuat, AccMagOrientation, headingfromMag, QuatToEuler, angle_between, QuatToRotMat, AxisAngleToRotMat, RotMatToQuat, AccMag2Euler
from math import atan2, atan
from numpy.linalg import inv
from numpy import linalg as LA
# import euclid



# read_imu = True
# visual_imu = True
read_imu = False
visual_imu = False

save_count = 0 
read_op = True
visual_op = False


if read_imu: 

    # filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_static.txt', 'r')
    # filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_static_write.txt', 'w')

    # filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_tilt_about_y_axis.txt', 'r')
    # filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_tilt_about_y_axis_write.txt', 'w')

    # Path change required
    filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_movement.txt', 'r')
    filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_movement_write.txt', 'w')


    # filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_straight.txt', 'r')
    # filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_straight_write.txt', 'w')

    # filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/leon_tianwei_3dprinter.txt', 'r')
    # filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_leon_tianwei_3dprinter_write.txt', 'w')


    #filename2 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_long_trajectory.txt', 'r')
    #filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_long_trajectory_write.txt', 'w')

    offset_mx =  71.195
    offset_my =  -14.104999999999997
    offset_mz =  -52.065
    scale_mx =  1.2081367686786824
    scale_my =  0.8622694178249733
    scale_mz =  0.9876067108342398



    dt = 1/10


    if visual_imu: 
        pygame.init()

        screen = Screen(1600,400,scale=1.5)
        cube1 = Cube(40,30,60)
        cube2 = Cube(40,30,60)
        cube3 = Cube(40,30,60)
        cube4 = Cube(40,30,60)
        cube5 = Cube(40,30,60)

        q1 = Quaternion(1,0,0,0)
        q2 = Quaternion(1,0,0,0)
        q3 = Quaternion(1,0,0,0)
        q4 = Quaternion(1,0,0,0)
        q5 = Quaternion(1,0,0,0)

        p1 = Vector3(-400,0,0)
        p2 = Vector3(-200,0,0)
        p3 = Vector3(0,0,0)
        p4 = Vector3(200,0,0)
        p5 = Vector3(400,0,0)

        cube1.erase(screen)
        cube1.draw(screen,q1,p1)

        cube2.erase(screen)
        cube2.draw(screen,q2,p2)

        cube3.erase(screen)
        cube3.draw(screen,q3,p3)

        cube4.erase(screen)
        cube4.draw(screen,q4,p4)

        cube5.erase(screen)
        cube5.draw(screen,q5,p5)


        omega0 = [0,0,0]
        similaritywindowA3 = 0
        Sc = []
        Sg = []
        C = []
        G = []
        Eg = 0
        quatA3 = QuaternionClass(1, 0, 0, 0)
        quatMuseAlg = QuaternionClass(1, 0, 0, 0)

        similaritywindowMUSE = 0

        initial = 0
        update = 0

        Ax = []
        Ay = []
        Az = []

        beta = 0.80


        pygame.display.flip()


    for i in filename2: 
        save_count = save_count + 1
        print(save_count)

        sp = str(i).split(',')
        # print(sp)


        time = float(sp[0][2:].strip())

        # reads in g so multiply by 9.8
        ax = float(sp[1].strip())
        ay = float(sp[2].strip())
        az = float(sp[3].strip())


        ax = ax*9.8
        ay = ay*9.8
        az = az*9.8

        gx = float(sp[4].strip())*math.pi/180   #rad/s
        gy = float(sp[5].strip())*math.pi/180   #rad/s
        gz = float(sp[6].strip())*math.pi/180   #rad/s

        #uT 
        mx = float(sp[7].strip())
        my = float(sp[8].strip())
        mz = float(sp[9].strip())


        mx = mx - offset_mx
        my = my - offset_my
        mz = mz - offset_mz

        mx = mx*scale_mx
        my = my*scale_my
        mz = mz*scale_mz


        qw = float(sp[10].strip())
        qx = float(sp[11].strip())
        qy = float(sp[12].strip())
        qz = float(sp[13].strip())

        pitch = float(sp[14].strip())
        roll = float(sp[15].strip())
        yaw = float(sp[16].strip())


        if save_count >0: 
            # print(ax,ay,az, gx,gy,gz, mx,my,mz, qw, qx, qy, qz)
            print(ax,ay,az, gx,gy,gz, mx,my,mz, qw, qx, qy, qz, file = filename3)


        # print("yaw, pitch, roll: ", yaw, pitch, roll)


        heading = float(sp[17].split('\\r')[0].strip())
        # print("heading: ", heading)

        # print(computeheading(mx,my))
        # print(yaw, pitch, roll)

        accel = [ax, ay, az]
        gyro = [gx, gy, gz]
        mag = [mx, my, mz]

        # print(accel)

        a333 = 0



        # yawAM, pitchAM, rollAM, quatAM = AccMagOrientation(accel, mag)

        # print("ypr: ", yaw, pitch, roll)
        # print("ypr: ", yawAM, pitchAM, rollAM)

        # print("heading: ", heading)
        # print(headingM)

        # time_diff = 60

        if visual_imu: 
            if True:  #quaternion from imu 
                quat = QuaternionClass(qw, qx, qy, qz)

                q1.w = quat[0]
                q1.x = quat[1]
                q1.z = quat[2]
                q1.y = quat[3]

                

                q1 = q1.normalized()

                cube1.erase(screen)
                cube1.draw(screen,q1,p1)


            if True: #a3 

                q_a3 = 0 

                omega1 = [gx, gy, gz]
                quatG = IntegrationRK4(omega0, omega1, quatA3, dt)
                yawG, pitchG, rollG = QuatToEuler(quatG)

                if yawG < 0: 
                    yawG = -yawG*180/math.pi
                else:
                    yawG = 360 - yawG*180/math.pi

                # # print(yawG, pitchG, rollG)
                omega0 = omega1
         
            # #     # A3 Algorithm - accelerometer, magnetometer calibration
                # yawAM, pitchAM, rollAM, quatAM = AccMag2Euler(accel, mag)
                yawAM, pitchAM, rollAM, quatAM = androidAccMag2Euler(accel, mag)


            #   # print(yawAM, pitchAM, rollAM)
            # #     # TODO: Update quaternion if w < 240 degree, a < 2g 
                w = max(abs(np.array(gyro)))*180/math.pi
                a = max(abs(np.array(accel)))

            # #     # if w < 240 and a < 2*9.8:
            # #     #   print("stable")
            # #     # else:
            # #     #   print("moving")

            #   # headingM = headingfromMag(mag)
                headingM = computeheading(mx, my)
                # print("headingM:" , headingM)
                # print("heading: ", headingM)
                # print("yawG: ", yawG*180/math.pi)
            #   # print(headingM)


                if similaritywindowA3 > 1:
                    # print("similaritywindow")
                    # calculate pc and pg 
                    pc = 1/(2**np.var(np.subtract(Sc,C)))
                    pg = 1/(2**np.var(np.subtract(Sg,G)))
                    # print(pc)
                    # print(pg)
                    if pc > 0.2 and pg > 0.2: 
                        print("change?")
                        # TODO: if Ec < Eg, then update quaternion
                        E1 = -32.14*pc + 19.93
                        E2 = -12.86*pg + 11.57
                        Ec = max(E1, E2)
                        Eg = (Eg + 0.0003*w*dt + 0.001*a*dt)*1000
                        #print(Ec)
                        #print(Eg)
                        if Ec < Eg*1000:
            #               print(a333)
                            a333 = a333 + 1
                            print("A3 reset ")
                            q_a3 = 1
                            #quatA3 = quatAM
            #               #         quat = quatAM

                    # reset values
                    similaritywindowA3 = 0 
                    C = []
                    Sc = []
                    Sg = []
                    G = []
                    Eg = 0 
                else:
                    #     #TODO: update Eg 
                    Eg = Eg + 0.0003*w*dt + 0.001*a*dt
                    C.append(yawAM)
                    Sc.append(yawG)
                    Sg.append(rollG)
                    G.append(rollAM)
                    similaritywindowA3 = similaritywindowA3 + dt


                if q_a3: 
                    quatA3 = quatAM #QuaternionClass(quatAM[0], quatAM[1], quatAM[2], quatAM[3])
                    # print("quatAM", quatAM)
                else:
                    quatA3 = quatG 
                #   print("quatG", quatG[0], quatG[1], quatG[2], quatG[3])

                # print("quatA3", quatA3[0], quatA3[1], quatA3)

                yawA3, pitchA3, rollA3 = QuatToEuler(quatA3)
                # print("yawA3: ", yawA3*180/math.pi)


                quatA3_temp = QuaternionClass(quatA3[0], quatA3[1], quatA3[3], -quatA3[2])
                # quatA3 = quatA3_temp


                q3.w = quatA3_temp[0]
                q3.x = quatA3_temp[1]
                q3.y = quatA3_temp[2]
                q3.z = quatA3_temp[3]

                q3 = q3.normalized()
                cube3.erase(screen)
                cube3.draw(screen,q3,p3)

            if True: # MUSE 
            #   # # Initial yaw, pitch, roll from Accelerometer and Magnetometer 
                #yawAM, pitchAM, rollAM, quatAM = AccMag2Euler(accel, mag)
                yawAM, pitchAM, rollAM, quatAM = androidAccMag2Euler(accel, mag)

                omega1 = [gx, gy, gz]
                quatG = IntegrationRK4(omega0, omega1, quatMuseAlg, dt)
                yawG, pitchG, rollG = QuatToEuler(quatG)

                omega0 = omega1
                headingM = computeheading(mx, my)
                # headingM = headingfromMag(mag)

                if initial < 30:
                    quatMuseAlg = quatAM
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
                    quatMuseAlg = quatAM
                    # print("similaritywindow: ", similaritywindowMUSE)
                    if similaritywindowMUSE > 1:

                        # print("Ax: ", Ax)
                        # print("Ay: ", Ay)
                        # print("Az: ", Az)

                        aAx = abs(np.array(Ax))
                        aAy = abs(np.array(Ay))
                        aAz = abs(np.array(Az))

                        # print("Ax: ", aAx)
                        # print("Ay: ", aAy)
                        # print("Az: ", aAz)

                        agAx = aAx - 9.8
                        agAy = aAy - 9.8
                        agAz = aAz - 9.8

                        # print("agAx: ", agAx)
                        # print("agAy: ", agAy)
                        # print("agAz: ", agAz)

                        aagAx = abs(agAx)
                        aagAy = abs(agAy)
                        aagAz = abs(agAz)

                        # print("aagAx: ", aagAx)
                        # print("aagAy: ", aagAy)
                        # print("aagAz: ", aagAz)


                        x_max = max(aagAx)
                        y_max = max(aagAy)
                        z_max = max(aagAz)


                        # Ax = abs(abs(np.array(Ax))-9.8)
                        # Ay = abs(abs(np.array(Ax))-9.8)
                        # Az = abs(abs(np.array(Az))-9.8)
                        # # print(Az)               

                            
                        # # x_max = max([abs(max(Ax)), abs(min(Ax))])
                        # # y_max = max([abs(max(Ay)), abs(min(Ay))])
                        # # z_max = max([abs(max(Az)), abs(min(Az))])

                        # x_max = max(Ax)
                        # y_max = max(Ay)
                        # z_max = max(Az)

                        # print("x: ", x_max)
                        # print("y: ", y_max)
                        # print("z: ", z_max)

                        xyz_min = min([x_max, y_max, z_max])
                        # print(xyz_min)

                        # acceleration roughly measures 9.8m/s2


                        if xyz_min < 1: 
                            print("yes, update quat with AM")
                            Oa = QuatToRotMat(quatAM)
                            Og = QuatToRotMat(quatG)

                            Ocomp = np.mat(Oa)*(1-beta) + np.mat(Og)*beta
                            # print("Oa")
                            # print(Oa)
                            # print("Og")
                            # print(Og)
                            # print("Ocomp")
                            # print(Ocomp)
                            quatComp = RotMatToQuat(np.array(np.mat(Ocomp)))
                            quatMuseAlg = quatComp
                            update = 1
                            # Update 3D magnetic vector estimation
                        N_L = np.mat([[mx],[my],[mz]])
                            # print("N_L")
                            # print(N_L)
                        O = QuatToRotMat(quatAM)
                        N_G = O*N_L

                                                    # reset values
                        similaritywindowMUSE = 0 
                        Ax = []
                        Ay = []
                        Az = []

                    else:
                        Ax.append(ax)
                        Ay.append(ay)
                        Az.append(az)
                        similaritywindowMUSE = similaritywindowMUSE + dt
                                
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
                        quatMuseAlg = QuaternionClass(quatMUSE[0], quatMUSE[1], quatMUSE[2], quatMUSE[3])                

                        #print("update quat with MUSE")

                    update = 0

                    yawMUSE, pitchMUSE, rollMUSE = QuatToEuler(quatMuseAlg)
                    # print("yawMUSE: ", yawMUSE*180/math.pi)

                q4.w = quatMuseAlg[0]
                q4.x = quatMuseAlg[1]
                q4.y = quatMuseAlg[3]
                q4.z = -quatMuseAlg[2]

                

                q4 = q4.normalized()
                cube4.erase(screen)
                cube4.draw(screen,q4,p4)


            #   print("yaw: ", yaw )

            pygame.display.flip()
            pygame.time.delay(0)
            event = pygame.event.poll()
            if event.type == pygame.QUIT \
                or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                break



        

# from itertools import zip

# with filename as textfile1, filename2 as textfile2: 
#     for x, y in zip(textfile1, textfile2):
#         x = x.strip()
#         y = y.strip()
#         print("{0}\t{1}".format(x, y))



if read_op: 

    if visual_op:

        pygame.init()

        screen = Screen(1600,400,scale=1.5)
        cube1 = Cube(40,30,60)
        cube2 = Cube(40,30,60)
        cube3 = Cube(40,30,60)
        cube4 = Cube(40,30,60)
        cube5 = Cube(40,30,60)

        q1 = Quaternion(1,0,0,0)
        q2 = Quaternion(1,0,0,0)
        q3 = Quaternion(1,0,0,0)
        q4 = Quaternion(1,0,0,0)
        q5 = Quaternion(1,0,0,0)

        p1 = Vector3(-400,0,0)
        p2 = Vector3(-200,0,0)
        p3 = Vector3(0,0,0)
        p4 = Vector3(200,0,0)
        p5 = Vector3(400,0,0)

        cube1.erase(screen)
        cube1.draw(screen,q1,p1)

        cube2.erase(screen)
        cube2.draw(screen,q2,p2)

        cube3.erase(screen)
        cube3.draw(screen,q3,p3)

        cube4.erase(screen)
        cube4.draw(screen,q4,p4)

        cube5.erase(screen)
        cube5.draw(screen,q5,p5)



        pygame.display.flip()

    # filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_static.csv', 'r')
    # filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_static_write.txt', 'w')

    filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_movement.csv', 'r')
    filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_movement_write.txt', 'w')
    filename4 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_movement_position_write.txt','w')

    # filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_straight_12_070_1270.csv', 'r')
    # filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_straight_write.txt', 'w')
    # filename4 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_straight_position_write.txt','w')


    # filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/leon_tianwei_3dprinter_2439.csv', 'r')
    # filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_leon_tianwei_3dprinter_write.txt', 'w')
    # filename4 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_leon_tianwei_3dprinter_positions.txt','w')

    # filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_long_trajectory.csv', 'r')
    # filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_long_trajectory_write.txt', 'w')
    # filename4 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_long_trajectory_positions.txt','w')





    skip_lines = 0 
    count = 0 
    for i in filename: 
        if skip_lines < 127: 
            # print(skip_lines)
            pass

        else:

            if len(str(i))> 300: 
                # print(len(str(i)))
                # print("ho")

                sp = str(i).split(',')

                frame = sp[0]
                print(frame)

                op_qw = float(sp[5])
                op_qx = float(sp[2])
                op_qy = float(sp[3])
                op_qz = float(sp[4])


                time = float(sp[1])
                # print(time)
                op_px = float(sp[6])
                op_py = float(sp[7])
                op_pz = float(sp[8])

                # print(op_qw, op_qx, op_qy, op_qz)
                print(time, op_px, op_py, op_pz, file = filename4)

                quat = QuaternionClass(op_qw, op_qx, op_qy, op_qz)
                quat = qnormalized(quat)
                # print(quat[0], quat[1], quat[2], quat[3])

                if visual_op:
                    q5.w = quat[0]
                    q5.x = quat[1]
                    q5.y = quat[2]
                    q5.z = quat[3]

                print(quat[0], quat[1], quat[2], quat[3], file = filename1)

                
                if visual_op:
                    cube5.erase(screen)
                    cube5.draw(screen,q5,p5)

                    pygame.display.flip()
                    pygame.time.delay(0)

            else:
                print(str(i))
        if visual_op:
            event = pygame.event.poll()
            if event.type == pygame.QUIT \
                or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                break
        skip_lines = skip_lines + 1
        


