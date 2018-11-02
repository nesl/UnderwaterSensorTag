from mpulib import computeheading, attitudefromCompassGravity, RP_calculate, MadgwickQuaternionUpdate, Euler2Quat, quaternion_to_euler_angle, MPU9250_computeEuler
import socket, traceback
import csv
import struct
import sys, time, string, pygame

import pygame
import pygame.draw
import pygame.time
import numpy as np
from math import sin, cos, acos
from euclid import Vector3, Quaternion
from EuclidObjects import Cube, Screen, Grid, PerspectiveScreen

import math

# from pygame.locals import *
# from ponycube import *
from madgwickahrs import *
import quaternion
from quaternion import QuaternionClass
from a3muse import androidAccMag2Euler, qnormalized, quatNormalized, IntegrationRK4, EulerToQuat, AccMagOrientation, headingfromMag, QuatToEuler, angle_between, QuatToRotMat, AxisAngleToRotMat, RotMatToQuat, AccMag2Euler
from math import atan2, atan
from numpy.linalg import inv
from numpy import linalg as LA
import matplotlib.pyplot as plt

# import euclid

# filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_static_write.txt', 'r')
# filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_static_write.txt', 'r')

# filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_movement_write.txt', 'r')
# filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_movement_write.txt', 'r')


# filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_straight_write.txt', 'r')
# filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_straight_write.txt', 'r')

# filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_values.txt', 'w')
# filenameOptitrack = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_Optitrack.txt', 'w')
# filenameMadgwick = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_Madgwick.txt', 'w')
# filenameA3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_A3.txt', 'w')
# filenameMUSE = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_MUSE.txt', 'w')
# filenameES = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_ES.txt', 'w')



# filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_leon_tianwei_3dprinter_write.txt', 'r')
# filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_leon_tianwei_3dprinter_write.txt', 'r')

# filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_values.txt', 'w')
# filenameOptitrack = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_Optitrack.txt', 'w')
# filenameMadgwick = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_Madgwick.txt', 'w')
# filenameA3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_A3.txt', 'w')
# filenameMUSE = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_MUSE.txt', 'w')
# filenameES = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_ES.txt', 'w')


filename3 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/imu_long_trajectory_write.txt', 'r')
filename1 = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_long_trajectory_write.txt', 'r')
filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_long_trajectory_values.txt', 'w')





dt = 1/10
pygame.init()
screen = Screen(1600,600,scale=1.5)

cube0 = Cube(40,30,60)
cube1 = Cube(40,30,60)
cube2 = Cube(40,30,60)
cube3 = Cube(40,30,60)
cube4 = Cube(40,30,60)
cube5 = Cube(40,30,60)

q0 = Quaternion(1,0,0,0)
q1 = Quaternion(1,0,0,0)
q2 = Quaternion(1,0,0,0)

# q = QuaternionClass(0,0,-1,0)
# dq = QuaternionClass(0,0,-1,0)
# q = q * dq

# q2.w = q[0]
# q2.x = q[1]
# q2.y = q[2]
# q2.z = q[3]			
# q2 = q2.normalized()


q3 = Quaternion(1,0,0,0)


# q3.w = q[0]
# q3.x = q[1]
# q3.y = q[2]
# q3.z = q[3]			
# q3 = q3.normalized()


q4 = Quaternion(1,0,0,0)
q5 = Quaternion(1,0,0,0)

p0 = Vector3(0,-100,0)
p1 = Vector3(-400,100,0)
p2 = Vector3(-200,100,0)
p3 = Vector3(0,100,0)
p4 = Vector3(200,100,0)
p5 = Vector3(400,100,0)

cube0.erase(screen)
cube0.draw(screen,q0,p0)


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

dq = QuaternionClass(0,0,-1,0)


# Madgwick
Imupredict = MadgwickAHRS();
Imupredict2 = MadgwickAHRS(); 


#ES
omega0_es = [0,0,0]
similaritywindowES = 0
Sc_es = []
Sg_es = []
C_es = []
G_es = []
Eg_es = 0
quatES = QuaternionClass(1,0,0,0)
initialES = 0
esss = 0 

Ax_es = []
Ay_es = []
Az_es = []


# A3 
omega0 = [0,0,0]
similaritywindowA3 = 0
Sc = []
Sg = []
C = []
G = []
Eg = 0
quatA3 = QuaternionClass(1,0,0,0)
a333 = 0



quatMuseAlg = QuaternionClass(1, 0, 0, 0)

similaritywindowMUSE = 0

initial = 0
update = 0

Ax = []
Ay = []
Az = []

beta = 0.80

quat = QuaternionClass(1,0,0,0)
# 1 Hz - 1000
# 10 Hz - 100



initialES = 0 
updateES = 0 
AxES = []
AyES = []
AzES = []
similaritywindowES = 0
ImupredictES = MadgwickAHRS();


count = 0 

with filename3 as textfile1, filename1 as textfile2: 
	for x, y in zip(textfile1, textfile2):


		print(count)
		count = count + 1 
		if count < 0:
			pass
		else:

			imu_sp = x.split()
			ax = float(imu_sp[0])
			ay = float(imu_sp[1])
			az = float(imu_sp[2])


			gx = float(imu_sp[3])
			gy = float(imu_sp[4])
			gz = float(imu_sp[5])


			mx = float(imu_sp[6])
			my = float(imu_sp[7])
			mz = float(imu_sp[8])

			# rax = ax
			# ray = ay 
			# raz = az

			# rgx = gx
			# rgy = gy 
			# rgz = gz

			# rmx = mx 
			# rmy = my 
			# rmz = mz 

			# ax = rax
			# ay = raz
			# az = ray

			# gx = rgx
			# gy = rgz
			# gz = rgy

			# mx = rmx
			# my = rmz
			# mz = rmy


			qw = float(imu_sp[9])
			qx = float(imu_sp[10])
			qy = float(imu_sp[11])
			qz = float(imu_sp[12])

			op_sp = y.split()
			op_qw = float(op_sp[0])
			op_qx = float(op_sp[1])
			op_qy = float(op_sp[2])
			op_qz = float(op_sp[3])

			# print(ax,ay,az, gx,gy,gz, mx,my,mz, qw, qx, qy, qz)


			accel = [ax, ay, az]
			gyro = [gx, gy, gz]
			mag = [mx, my, mz]



			# Optitrack 
			if True: 

				op_q = QuaternionClass(op_qw, op_qz, op_qy, op_qx)# *QuaternionClass(np.sqrt(0.5),0,np.sqrt(0.5),0)

				q0.w = op_q[0]
				q0.x = op_q[1]
				q0.y = op_q[3]
				q0.z = op_q[2]

				q0 = q0.normalized()

				yawOP, pitchOP, rollOP = QuatToEuler([q0.w, q0.x, q0.y, q0.z])
				# print(yawOP, pitchOP, rollOP, file = filenameOptitrack)
				# print("quaternion: ", q0.w, q0.x, q0.y, q0.z )
				# print("quattoeuler: ", QuatToEuler([q0.w, q0.x, q0.y, q0.z]))

				cube0.erase(screen)
				cube0.draw(screen,q0,p0)



			if True: # Quaternion form IMU

				quat = QuaternionClass(qw, qx, qy, qz) 
				q1.w = quat[0]
				q1.x = quat[1]
				q1.y = quat[2]
				q1.z = quat[3]

				q1 = q1.normalized()

				cube1.erase(screen)
				cube1.draw(screen,q1,p1)

			if True: 
				Imupredict.samplePeriod = dt#0.1
				Imupredict.update(gyro,accel,mag)
				quatMad = Imupredict.quaternion
				quatMad = qnormalized(quatMad)
				Imupredict.quaternion = quatMad
				#quatMad = quatNormalized(quatMad)
			
				q2.w = quatMad[0]
				q2.x = quatMad[1]
				q2.y = quatMad[2]
				q2.z = quatMad[3]
				q2 = q2.normalized()

				yawMad, pitchMad, rollMad = QuatToEuler([q2.w, q2.x, q2.y, q2.z])
				# print(yawMad, pitchMad, rollMad, file = filenameMadgwick)

				cube2.erase(screen)
				cube2.draw(screen,q2,p2)


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
		 
			# # 	# A3 Algorithm - accelerometer, magnetometer calibration
				# yawAM, pitchAM, rollAM, quatAM = AccMag2Euler(accel, mag)
				yawAM, pitchAM, rollAM, quatAM = androidAccMag2Euler(accel, mag)


			# 	# print(yawAM, pitchAM, rollAM)
			# # 	# TODO: Update quaternion if w < 240 degree, a < 2g 
				w = max(abs(np.array(gyro)))*180/math.pi
				a = max(abs(np.array(accel)))

				if w < 240 and a < 2*9.8:
				# # 	# if w < 240 and a < 2*9.8:
				# # 	# 	print("stable")
				# # 	# else:
				# # 	# 	print("moving")

				# 	# headingM = headingfromMag(mag)
					headingM = computeheading(mx, my)

					# print("headingM:" , headingM)
					# print("heading: ", headingM)
					# print("yawG: ", yawG*180/math.pi)
				# 	# print(headingM)


					if similaritywindowA3 > 2:
						# print("similaritywindow")
						# calculate pc and pg 
						pc = 1/(2**np.var(np.subtract(Sc,C)))
						pg = 1/(2**np.var(np.subtract(Sg,G)))
						# print("pc: ",pc)
						# print("pg: ", pg)
						if pc > 0.2 and pg > 0.2: 
							# print("mag: ", mag)
							print("change?")
							# TODO: if Ec < Eg, then update quaternion
							E1 = -32.14*pc + 19.93
							E2 = -12.86*pg + 11.57
							Ec = max(E1, E2)
							Eg = (Eg + 0.0003*w*dt + 0.001*a*dt)*1000
							# print(Ec)
							# print(Eg)
							if Ec < Eg*1000:
				# 				print(a333)
								a333 = a333 + 1
								print("A3 reset ")
								q_a3 = 1
								#quatA3 = quatAM
				# 				#         quat = quatAM

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
					#	print("quatG", quatG[0], quatG[1], quatG[2], quatG[3])

					# print("quatA3", quatA3[0], quatA3[1], quatA3)

					yawA3, pitchA3, rollA3 = QuatToEuler(quatA3)
					# print("yawA3: ", yawA3*180/math.pi)


					quatA3_temp = QuaternionClass(quatA3[0], quatA3[1], quatA3[2], quatA3[3])
					# quatA3_temp = QuaternionClass(quatA3[0], quatA3[1], quatA3[3], -quatA3[2])
					# quatA3 = quatA3_temp


					q3.w = quatA3_temp[0]
					q3.x = quatA3_temp[1]
					q3.y = quatA3_temp[2]
					q3.z = quatA3_temp[3]

					q3 = q3.normalized()

					yawA3, pitchA3, rollA3 = QuatToEuler([q3.w, q3.x, q3.y, q3.z])
					# print(yawA3, pitchA3, rollA3, file = filenameA3)

					cube3.erase(screen)
					cube3.draw(screen,q3,p3)

			if True: # MUSE 
			# 	# # Initial yaw, pitch, roll from Accelerometer and Magnetometer 
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
					if similaritywindowMUSE > 2:

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

				# q4.w = quatMuseAlg[0]
				# q4.x = quatMuseAlg[1]
				# q4.y = quatMuseAlg[3]
				# q4.z = -quatMuseAlg[2]
				q4.w = quatMuseAlg[0]
				q4.x = quatMuseAlg[1]
				q4.y = quatMuseAlg[2]
				q4.z = quatMuseAlg[3]


				q4 = q4.normalized()
				yawMUSE, pitchMUSE, rollMUSE = QuatToEuler([q4.w, q4.x, q4.y, q4.z])
				# print(yawMUSE, pitchMUSE, rollMUSE, file = filenameMUSE)

				cube4.erase(screen)
				cube4.draw(screen,q4,p4)

			if True:

				omega1 = [gx, gy, gz]
				quatG = IntegrationRK4(omega0, omega1, quatES, dt)
				yawG, pitchG, rollG = QuatToEuler(quatG)
				omega0 = omega1

				ImupredictES.samplePeriod = dt 
				ImupredictES.update(gyro,accel,mag)
				quatMadES = ImupredictES.quaternion
				yawAMES, pitchAMES, rollAMES, quatAMES = androidAccMag2Euler(accel, mag)

				if initialES < 30:
					quatES = quatAMES
					print("initial")
					initialES = initialES + 1 
				else:
					if similaritywindowES > 2: 

						aAx = abs(np.array(AxES))
						aAy = abs(np.array(AyES))
						aAz = abs(np.array(AzES))

						agAx = aAx 
						agAy = aAy 
						agAz = aAz - 9.8

						aagAx = abs(agAx)
						aagAy = abs(agAy)
						aagAz = abs(agAz)
						# print(aagAz)

						x_max = max(aagAx)
						y_max = max(aagAy)
						z_max = max(aagAz)

						# print(z_max)
						if z_max < 1: 
							updateES = 1
						


						similaritywindowES = 0 
						AxES = []
						AyES = []
						AzES = []

					else: 
						AxES.append(ax)
						AyES.append(ay)
						AzES.append(az)
						similaritywindowES = similaritywindowES + dt

					if updateES: 
						quatES = quatAMES
						print("update with quatES")
					else:
						quatES = quatG
					updateES = 0


				quatES = qnormalized(quatES)

				ImupredictES.quaternion = quatES

				q5.w = quatES[0]
				q5.x = quatES[1]
				q5.y = quatES[2]
				q5.z = quatES[3]
					
				q5 = q5.normalized()
				yawES, pitchES, rollES = QuatToEuler([q5.w, q5.x, q5.y, q5.z])
				# print(yawES, pitchES, rollES, file = filenameES)
				cube5.erase(screen)
				cube5.draw(screen,q5,p5)




			if False: #a3 

				q_es = 0 

				omega1_es = [gx, gy, gz]
				quatG = IntegrationRK4(omega0_es, omega1_es, quatES, dt)
				yawG, pitchG, rollG = QuatToEuler(quatG)

				if yawG < 0: 
					yawG = -yawG*180/math.pi
				else:
					yawG = 360 - yawG*180/math.pi

				# # print(yawG, pitchG, rollG)
				omega0_es = omega1_es
		 
			# # 	# A3 Algorithm - accelerometer, magnetometer calibration
				# yawAM, pitchAM, rollAM, quatAM = AccMag2Euler(accel, mag)
				yawAM, pitchAM, rollAM, quatAM = androidAccMag2Euler(accel, mag)




				if initialES < 30:
					quatES = quatAM
					print("initialES")

					initialES = initialES + 1
				else:




				# 	# print(yawAM, pitchAM, rollAM)
				# # 	# TODO: Update quaternion if w < 240 degree, a < 2g 
					w = max(abs(np.array(gyro)))*180/math.pi
					a = max(abs(np.array(accel)))

					if w < 240 and a < 2*9.8:
					# # 	# if w < 240 and a < 2*9.8:
					# # 	# 	print("stable")
					# # 	# else:
					# # 	# 	print("moving")

					# 	# headingM = headingfromMag(mag)
						headingM = computeheading(mx, my)

						# print("headingM:" , headingM)
						# print("heading: ", headingM)
						# print("yawG: ", yawG*180/math.pi)
					# 	# print(headingM)


						if similaritywindowES > 2:



							aAx_es = abs(np.array(Ax_es))
							aAy_es = abs(np.array(Ay_es))
							aAz_es = abs(np.array(Az_es))

							agAx_es = aAx_es - 9.8
							agAy_es = aAy_es - 9.8
							agAz_es = aAz_es - 9.8

							aagAx_es = abs(agAx_es)
							aagAy_es = abs(agAy_es)
							aagAz_es = abs(agAz_es)

							x_max_es = max(aagAx_es)
							y_max_es = max(aagAy_es)
							z_max_es = max(aagAz_es)

							xyz_min_es = min([x_max_es, y_max_es, z_max_es])

							if xyz_min_es < 1: 
								print("update with AM _ES ")
								q_es = 1
				

							# print("similaritywindow")
							# calculate pc and pg 
							pc_es = 1/(2**np.var(np.subtract(Sc_es,C_es)))
							pg_es = 1/(2**np.var(np.subtract(Sg_es,G_es)))
							# print("pc_es: ",pc_es)
							# print("pg_es: ", pg_es)
							if pc_es > 0.2 and pg_es > 0.2: 
								# print("mag: ", mag)
								print("change?")
								# TODO: if Ec < Eg, then update quaternion
								E1_es = -32.14*pc_es + 19.93
								E2_es = -12.86*pg_es + 11.57
								Ec_es = max(E1_es, E2_es)
								Eg_Es = (Eg_es + 0.0003*w*dt + 0.001*a*dt)*1000
								# print(Ec_es)
								# print(Eg_es)
								if Ec_es < Eg_es*1000:
					# 				print(a333)
									esss = esss + 1
									print("ES reset ")
									q_es = 1
									#quatA3 = quatAM
					# 				#         quat = quatAM

							# reset values
							similaritywindowES = 0 
							C_es = []
							Sc_es = []
							Sg_es = []
							G_es = []
							Eg_es = 0 
							Ax_es = []
							Ay_es = []
							Az_es = []
						else:
							Ax_es.append(ax)
							Ay_es.append(ay)
							Az_es.append(az)
							#     #TODO: update Eg 
							Eg_es = Eg_es + 0.0003*w*dt + 0.001*a*dt
							C_es.append(yawAM)
							Sc_es.append(yawG)
							Sg_es.append(rollG)
							G_es.append(rollAM)
							similaritywindowES = similaritywindowES + dt


						if q_es: 
							quatES = quatAM #QuaternionClass(quatAM[0], quatAM[1], quatAM[2], quatAM[3])
							# print("quatAM", quatAM)
						else:
							quatES = quatG 
						#	print("quatG", quatG[0], quatG[1], quatG[2], quatG[3])

						# print("quatA3", quatA3[0], quatA3[1], quatA3)

					yawES, pitchES, rollES = QuatToEuler(quatES)
					# print("yawA3: ", yawA3*180/math.pi)


				quatES_temp = QuaternionClass(quatES[0], quatES[1], quatES[3], -quatES[2])
					# quatA3 = quatA3_temp


				q5.w = quatES_temp[0]
				q5.x = quatES_temp[1]
				q5.y = quatES_temp[2]
				q5.z = quatES_temp[3]

				q5 = q5.normalized()
				cube5.erase(screen)
				cube5.draw(screen,q5,p5)

			print(ax, ay, az, gx, gy, gz, mx, my, mz, q0.w, q0.x, q0.y, q0.z, q1.w, q1.x, q1.y, q1.z, q2.w, q2.x, q2.y, q2.z, q3.w, q3.x, q3.y, q3.z, q4.w, q4.x, q4.y, q4.z, q5.w, q5.x, q5.y, q5.z, file = filenameIMU)

			# print(ax, ay, az, gx, gy, gz, mx, my, mz, yawOP, pitchOP, rollOP, yawMad, pitchMad, rollMad, yawA3, pitchA3, rollA3, yawMUSE, pitchMUSE, rollMUSE, yawES, pitchES, rollES, file = filenameIMU)


			pygame.display.flip()
			pygame.time.delay(0)
			event = pygame.event.poll()
			if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
				break






			
		  







