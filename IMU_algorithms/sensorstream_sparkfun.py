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
# import euclid


import serial 
ser = serial.Serial('/dev/tty.usbmodem14411')
ser.baudrate = 115200
ser.timeout = 3
prev_time = 0 


offset_mx = 77.345 
offset_my = -13.725
offset_mz = -71.64

scale_mx = 1.1
scale_my = 1.13
scale_mz = 0.827


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

incr = Quaternion(0.96,0.01,0.01,0).normalized()
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


# Madgwick
Imupredict = MadgwickAHRS();
Imupredict2 = MadgwickAHRS(); 

# A3 
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

quat = QuaternionClass(1,0,0,0)
# 1 Hz - 1000
# 10 Hz - 100


while True: 
	reading = ser.readline()
	# print(reading)
	sp = str(reading).split(',')
	# print(sp)


	time = float(sp[0][2:].strip())

	# reads in g so multiply by 9.8
	ax = float(sp[1].strip())
	ay = float(sp[2].strip())
	az = float(sp[3].strip())


	ax = ax*9.8
	ay = ay*9.8
	az = az*9.8

	gx = float(sp[4].strip())*math.pi/180	#rad/s
	gy = float(sp[5].strip())*math.pi/180	#rad/s
	gz = float(sp[6].strip())*math.pi/180	#rad/s

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



	# print("yaw, pitch, roll: ", yaw, pitch, roll)


	heading = float(sp[17].split('\\r')[0].strip())

	# print(heading)

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

	if True:  #quaternion from imu 
		# yellow area facing straight if imu hold with usbside facing me
		# print("yaw: ", yaw)
		q1w = float(sp[10].strip())
		q1x = float(sp[11].strip())
		q1z = -float(sp[12].strip())
		q1y = float(sp[13].split('\\r')[0].strip())

		# quatMDP = QuaternionClass(q1w, q1x, q1y, q1z)
		# rollMDP, pitchMDP, yawMDP = QuatToEuler(quatMDP)
		# print("yawMDP: ", yawMDP)

		q1.w = q1w
		q1.x = q1x
		q1.z = q1z
		q1.y = q1y

		q1 = q1.normalized()

		cube1.erase(screen)
		cube1.draw(screen,q1,p1)

		print("yaw: ", yaw )



	if True: # Madgwick Algorithm 
		Imupredict.samplePeriod = 0.1
		Imupredict.update(gyro,accel,mag)
		quatMad = Imupredict.quaternion
		quatMad = qnormalized(quatMad)
		Imupredict.quaternion = quatMad
		#quatMad = quatNormalized(quatMad)

		yawMad, pitchMad, rollMad = QuatToEuler(quatMad)
		print("yawMad: ", yawMad*180/math.pi)
		
		q2.w = quatMad[0]
		q2.x = quatMad[1]
		q2.z = -quatMad[2]
		q2.y = quatMad[3]
		

		q2 = q2.normalized()
		cube2.erase(screen)
		cube2.draw(screen,q2,p2)

	if False: 


		# quat =  MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, quat)
		# q5.w = quat[0]
		# q5.x = quat[1]
		# q5.z = -quat[2]
		# q5.y = quat[3]
		

		# q5 = q5.normalized()
		# cube5.erase(screen)
		# cube5.draw(screen,q5,p5)




		yawT, pitchT, rollT, quatT = androidAccMag2Euler(accel, mag)
		if yawT > 0: 
			yawT = 360 - yawT*180/math.pi
		else: 
			yawT = -yawT*180/math.pi
		# print("yaw: ",yawT)
		q5.w = quatT[0]
		q5.x = quatT[1]
		q5.z = -quatT[2]
		q5.y = quatT[3]
		

		q5 = q5.normalized()
		cube5.erase(screen)
		cube5.draw(screen,q5,p5)

	# 	Imupredict2.samplePeriod = 0.1
	# 	Imupredict2.update_imu(gyro,accel)
	# 	quatMad2 = Imupredict2.quaternion
	# 	quatMad2 = qnormalized(quatMad)
	# 	Imupredict2.quaternion = quatMad2

	# 	q5.w = quatMad2[0]
	# 	q5.x = quatMad2[1]
	# 	q5.z = -quatMad2[2]
	# 	q5.y = quatMad2[3]
		

	# 	q5 = q5.normalized()
	# 	cube5.erase(screen)
	# 	cube5.draw(screen,q5,p5)

# https://stackoverflow.com/questions/32372847/android-algorithms-for-sensormanager-getrotationmatrix-and-sensormanager-getori/35390001#35390001



	if True: #a3 

		q_a3 = 0 

		omega1 = [gx, gy, gz]
		dt = 0.1
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

	# # 	# if w < 240 and a < 2*9.8:
	# # 	# 	print("stable")
	# # 	# else:
	# # 	# 	print("moving")

	# 	# headingM = headingfromMag(mag)
		headingM = computeheading(mx, my)
		# print("heading: ", headingM)
		# print("yawG: ", yawG*180/math.pi)
	# 	# print(headingM)


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
		print("yawA3: ", yawA3*180/math.pi)


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
	# 	# # Initial yaw, pitch, roll from Accelerometer and Magnetometer 
		#yawAM, pitchAM, rollAM, quatAM = AccMag2Euler(accel, mag)
		yawAM, pitchAM, rollAM, quatAM = androidAccMag2Euler(accel, mag)

		omega1 = [gx, gy, gz]
		dt = 0.1
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
			print("yawMUSE: ", yawMUSE*180/math.pi)

		q4.w = quatMuseAlg[0]
		q4.x = quatMuseAlg[1]
		q4.y = quatMuseAlg[3]
		q4.z = -quatMuseAlg[2]

		

		q4 = q4.normalized()
		cube4.erase(screen)
		cube4.draw(screen,q4,p4)

	if True: 

		# quatDMP = QuaternionClass(qw, qx, qy, qz)


		# yawDMP, pitchDMP, rollDMP = MPU9250_computeEuler(qw, qx, qy, qz)
		# print("yprDMP: ", yawDMP, pitchDMP, rollDMP)
		# # print("ypr: ", yaw, pitch, roll)

		# quatDMP1 = Euler2Quat(yawDMP, pitchDMP, rollDMP)



		# quatDMP = qnormalized(quatDMP)
		# print("quatDMP: " , quatDMP[0], quatDMP[1], quatDMP[2], quatDMP[3])

		# yawDMP, pitchDMP, rollDMP = quaternion_to_euler_angle(quatDMP[0], quatDMP[1], quatDMP[2], quatDMP[3])

		# quatDMP1 = Euler2Quat(yawDMP, pitchDMP, rollDMP)


		# quatDMP1 = qnormalized(quatDMP1)

		# print("quatDMP1: ", quatDMP1[0], quatDMP1[1], quatDMP1[2], quatDMP1[3])





		# print("ypr: ", yawDMP*180/math.pi)

		# if yaw - 180 > 0 : 
		# 	yaw -= 360
		# yaw *= math.pi/180


		# if roll - 180 > 0 : 
		# 	roll -= 360
		# roll *= math.pi/180

		# if pitch - 180 > 0 : 
		# 	pitch -= 360
		# pitch *= math.pi/180

		# quatDMP = Euler2Quat(yaw, pitch, roll)




		# quatDMP = qnormalized(quatDMP)

		# q5.w = quatDMP1[0]
		# q5.x = quatDMP1[1]
		# q5.y = quatDMP1[3]
		# q5.z = -quatDMP1[2]		

		# yawES = math.atan2(mx,my)
		# rollES, pitchES = RP_calculate(accel)
		# rollES = rollES


		# yawES *= 180/math.pi
		# if yawES < 0 :
		# 	yawES += 360.0

		# rollES *= 180/math.pi
		# if rollES < 0 :
		# 	rollES += 360.0


		# pitchES *= 180/math.pi
		# if pitchES < 0 :
		# 	pitchES += 360.0

		# print("yaw, yawES: ", yaw, yawES)
		# print("roll, rollES: ", roll, rollES)
		# print("pitch, pitchES: ", pitch, pitchES)




		# rollES = rollES * 180/math.pi
		# if rollES < 0: 
		# 	rollES = 360 + rollES 

		# 	rollES = (360 - rollES*180/math.pi)

		# rollES = rollES * math.pi/180

		# yawES = yawES*math.pi/180
		# rollES = rollES*math.pi/180

		# print("yawES: ", yawES)
# 


		# quatES = Euler2Quat(yaw*math.pi/180, pitch*math.pi/180, roll*math.pi/180)


		# # quatES = Euler2Quat(yawES*math.pi/180, 0, 0)
		# quatES = qnormalized(quatES)
		# # print("quatES: ", quatES[0], quatES[1], quatES[2], quatES[3])		# 3 - yaw 


		# q5.w = quatES[0]
		# q5.x = quatES[1]
		# q5.z = -quatES[2]
		# q5.y = quatES[3]
		

		q5 = q5.normalized()


		cube5.erase(screen)
		cube5.draw(screen,q5,p5)




	pygame.display.flip()
	pygame.time.delay(0)
	event = pygame.event.poll()
	if event.type == pygame.QUIT \
		or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
		break

	# print(time)
	# print(time-prev_time)

	# print(ax)
	# print(ay)
	# print(az)

	# print(gx)
	# print(gy)
	# print(gz)

	# print(mx)
	# print(my)
	# print(mz)





	# sp = reading.split()
	# print(float(sp[0][:-1]))

	# print(sp[1].split(','))
	# # print(float(sp[1][:-1]))



