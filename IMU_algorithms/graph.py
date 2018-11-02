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
from mpl_toolkits.mplot3d import Axes3D




# Optitrack position file 
# filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_straight_position_write.txt','r')
filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_leon_tianwei_3dprinter_positions.txt','r')
# filename = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/optitrack/optitrack_long_trajectory_positions.txt','r')

# filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_straight_values.txt', 'r')
filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_leontianwei3d_values.txt', 'r')
# filenameIMU = open('/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_algorithms/YPRFiles/imu_long_trajectory_values.txt', 'r')



px = []
py = []
pz = []
skip_lines = 0 

for i in filename: 
	if skip_lines < 0: 
		pass 
	else:
		sp = i.split()
		time = float(sp[0])
		px.append(float(sp[1]))
		py.append(float(sp[2]))
		pz.append(float(sp[3]))
	skip_lines = skip_lines + 1


# print(len(px))

short_len = len(px)

# fig = plt.figure(figsize=(15,10))
# axplot = fig.gca(projection='3d')
# axplot.set_title("Trajectory plotted from ground truth position")
# axplot.scatter(px[0], pz[0], py[0], color='r')
# axplot.plot(px[:short_len], pz[:short_len], py[:short_len], label='ground truth position')
# axplot.legend()
# axplot.set_xlabel('X axis')
# axplot.set_ylabel('Y axis')
# axplot.set_zlabel('Z axis')
# plt.show()




ax = []
ay = []
az = []

gx = []
gy = []
gz = []

mx = []
my = []
mz = []

# yawOP = []
# pitchOP = []
# rollOP = []

# yawMad = [] 
# pitchMad = [] 
# rollMad = []

# yawA3 = []
# pitchA3 = []
# rollA3 = []

# yawMUSE = []
# pitchMUSE = []
# rollMUSE = []

# yawES = []
# pitchES = []
# rollES = []

# ax, ay, az, gx, gy, gz, mx, my, mz, yawOP, pitchOP, rollOP, yawMad, pitchMad, rollMad, yawA3, pitchA3, rollA3, yawMUSE, pitchMUSE, rollMUSE, yawES, pitchES, rollES
# ax, ay, az, gx, gy, gz, mx, my, mz, q0.w, q0.x, q0.y, q0.z, q1.w, q1.x, q1.y, q1.z, q2.w, q2.x, q2.y, q2.z, q3.w, q3.x, q3.y, q3.z, q4.w, q4.x, q4.y, q4.z, q5.w, q5.x, q5.y, q5.z


quatOP = []
quatIMU = []
quatMadgwick = []
quatA3 = []
quatMUSE = []
quatES = []

yawOP_ = []
pitchOP_ = []
rollOP_ = []

yawMUSE_ = []
pitchMUSE_ = []
rollMUSE_ = []

yawA3_ = []
pitchA3_ = []
rollA3_ = []

yawES_ = []
pitchES_ = []
rollES_ = []


r =0
#q0.w, q0.x, q0.y, q0.z, q1.w, q1.x, q1.y, q1.z, q2.w, q2.x, q2.y, q2.z, q3.w, q3.x, q3.y, q3.z, q4.w, q4.x, q4.y, q4.z, q5.w, q5.x, q5.y, q5.z

for i in filenameIMU: 
	sp = i.split()
	# print(r)
	r = r +1

	ax.append(float(sp[0]))
	ay.append(float(sp[1]))
	az.append(float(sp[2]))

	# print("ax:", float(sp[0]))

	gx.append(float(sp[3]))
	gy.append(float(sp[4]))
	gz.append(float(sp[5]))

	mx.append(float(sp[6]))
	my.append(float(sp[7]))
	mz.append(float(sp[8]))

	# optitrack
	q0w = float(sp[9])
	q0x = float(sp[10])
	q0y = float(sp[11])
	q0z = float(sp[12])

	quatOP.append(QuaternionClass(q0w, q0x, q0y, q0z))
	yawOP, pitchOP, rollOP = QuatToEuler([q0w, q0x, q0y, q0z])
	# print("yawA3:", yawA3)

	yawOP_.append(yawOP)
	pitchOP_.append(pitchOP)
	rollOP_.append(rollOP)


	# IMU 
	q1w = float(sp[13])
	q1x = float(sp[14])
	q1y = float(sp[15])
	q1z = float(sp[16])
	quatIMU.append(QuaternionClass(q1w, q1x, q1y, q1z))


	# Madgwick
	q2w = float(sp[17])
	q2x = float(sp[18])
	q2y = float(sp[19])
	q2z = float(sp[20])
	quatMadgwick.append(QuaternionClass(q2w, q2x, q2y, q2z))

	# A3
	q3w = float(sp[21])
	q3x = float(sp[22])
	q3y = float(sp[23])
	q3z = float(sp[24])
	quatA3.append(QuaternionClass(q3w, q3x, q3y, q3z))
	yawA3, pitchA3, rollA3 = QuatToEuler([q3w, q3x, q3y, q3z])
	# print("yawA3:", yawA3)

	yawA3_.append(yawA3)
	pitchA3_.append(pitchA3)
	rollA3_.append(rollA3)

	# # MUSE
	q4w = float(sp[25])
	q4x = float(sp[26])
	q4y = float(sp[27])
	q4z = float(sp[28])
	quatMUSE.append(QuaternionClass(q4w, q4x, q4y, q4z))
	yawMUSE, pitchMUSE, rollMUSE = QuatToEuler([q4w, q4x, q4y, q4z])
	yawMUSE_.append(yawMUSE)
	pitchMUSE_.append(pitchMUSE)
	rollMUSE_.append(rollMUSE)


	# ES
	q5w = float(sp[29])
	q5x = float(sp[30])
	q5y = float(sp[31])
	q5z = float(sp[32])
	quatES.append(QuaternionClass(q5w, q5x, q5y, q5z))	
	yawES, pitchES, rollES = QuatToEuler([q5w, q5x, q5y, q5z])	
	yawES_.append(yawES)
	pitchES_.append(pitchES)
	rollES_.append(rollES)

	# yawOP.append(float(sp[9]))
	# pitchOP.append(float(sp[10]))
	# rollOP.append(float(sp[11]))

	# yawMad.append(float(sp[12]))
	# pitchMad.append(float(sp[13]))
	# rollMad.append(float(sp[14]))

	# yawA3.append(float(sp[15]))
	# pitchA3.append(float(sp[16]))
	# rollA3.append(float(sp[17]))

	# yawMUSE.append(float(sp[18]))
	# pitchMUSE.append(float(sp[19]))
	# rollMUSE.append(float(sp[20]))

	# yawES.append(float(sp[21]))
	# pitchES.append(float(sp[22]))
	# rollES.append(float(sp[23]))


filenameIMU.close()

# print(pitchMUSE_)

# Get dx, dy, dz 
dx = np.diff(px)
dy = np.diff(py)
dz = np.diff(pz)

cumx = np.cumsum(dx)
cumy = np.cumsum(dy)
cumz = np.cumsum(dz)

gtx = []
gty = []
gtz = []

gtx.append(0)
gty.append(0)
gtz.append(0)

concat_len = 281

for i in range(concat_len-1):#len(cumx)):
	gtx.append(cumz[i])
	gty.append(cumx[i])
	gtz.append(cumy[i])


print(len(gtx))
print(len(ax))
print(len(yawA3_))

# fig = plt.figure(figsize=(15,10))
# axplot = fig.gca(projection='3d')
# axplot.set_title("Cumulative sum")
# axplot.scatter(cumz[0], cumx
# 	[0], cumy[0], color='r')
# axplot.plot(gtx,gty,gtz, label='ground truth position')
# # axplot.plot(cumz,cumx,cumy, label='long truth position')

# axplot.legend()
# axplot.set_xlabel('X axis')
# axplot.set_ylabel('Y axis')
# axplot.set_zlabel('Z axis')
# plt.show()





# IMU only dead reckoning 

accel = [ax, ay, az]
gyro = [gx, gy, gz]
mag = [mx, my, mz]





dt = 0.1
# # No Kalman filter: IMU only dead reckoning

noKX = []
noKY = []
noKZ = []
p_x = 0
p_y = 0
p_z = 0
noKX.append(p_x)
noKY.append(p_y)
noKZ.append(p_z)


px = 0
py = 0
pz = 0 
vx = 0
vy = 0 
vz = 0 

k = 10
yaw = yawOP_[k]# yawMUSE_[5]
roll = rollOP_[k]
pitch = pitchOP_[k]



for i in range(len(ax)):
	
# ######### Prediction STEP starts ####################################################
	# position = position + veclocity*dt 

	# print(x[0], x[1], x[2], x[3], x[4], x[5])
	# x[0] = x[0] + x[3]*dt
	# x[1] = x[1] + x[4]*dt
	# x[2] = x[2] + x[5]*dt

	px = px + vx*dt 
	py = py + vy*dt 
	pz = pz + vz*dt
	
	# print("pxyz: ", px, py, pz )

	# x[6]- yaw, x[7] - pitch, x[8]- roll 
	# 
	# sa = sin(x[6]) # sin (yaw + yawrate)
	# ca = cos(x[6]) # cos (yaw + yawrate)
	# sb = sin(x[8]) # sin (roll + rollrate)
	# cb = cos(x[8]) # cos (roll + rollrate
	# sh = sin(x[7]) # sin (pitch + pitchrate)
	# ch = cos(x[7]) # cos (pitch + pitchrate)

	sa = sin(yaw) # sin (yaw + yawrate)
	ca = cos(yaw) # cos (yaw + yawrate)
	sb = sin(roll) # sin (roll + rollrate)
	cb = cos(roll) # cos (roll + rollrate
	sh = sin(pitch) # sin (pitch + pitchrate)
	ch = cos(pitch) # cos (pitch + pitchrate)
	# sa = sin(yawOP_[k]) # sin (yaw + yawrate)
	# ca = cos(yawOP_[k]) # cos (yaw + yawrate)
	# sb = sin(rollOP_[k]) # sin (roll + rollrate)
	# cb = cos(rollOP_[k]) # cos (roll + rollrate
	# sh = sin(pitchOP_[k]) # sin (pitch + pitchrate)
	# ch = cos(pitchOP_[k]) # cos (pitch + pitchrate)

	
	m00 = ch*ca
	m01 = -ch*sa*cb + sh*sb
	m02 = ch*sa*sb + sh*cb
	m10 = sa
	m11 = ca*cb
	m12 = -ca*sb
	m20 = -sh*ca
	m21 = sh*sa*cb + ch*sb
	m22 = -sh*sa*sb + ch*cb
	
	Axs = (m00*ax[i] + m01*ay[i] + m02*az[i])/9.8
	Ays = (m10*ax[i] + m11*ay[i] + m12*az[i])/9.8
	Azgs = (m20*ax[i] + m21*ay[i] + m22*az[i])

	# print(Axs, Ays, Azgs)
	Azs = (m20*ax[i] + m21*ay[i] + m22*az[i] - 9.8)/9.8
	
	# print("Axyz: ", Axs, Ays, Azs )
	vx = vx + Axs*dt
	vy = vy + Ays*dt
	vz = vz + Azs*dt

	# print("Vxyz: ", vx, vy, vz )

	# # velcotiy = velocity + acceleration * dt 
	# x[3] = x[3] + Axs*dt
	# x[4] = x[4] + Ays*dt
	# x[5] = x[5] + Azs*dt

	yaw = yaw + gz[i]*dt
	pitch = pitch + gy[i]*dt
	roll = roll + gz[i] *dt

	# yaw = yawES_[i]
	# pitch = pitchES_[i]
	# roll = rollES_[i]
	# yaw = yawOP_[i]
	# pitch = pitchOP_[i]
	# roll = rollOP_[i]
	# yaw = yawA3_[i]
	# pitch = pitchA3_[i]
	# roll = rollA3_[i]

	# print("YPR: ", yaw, pitch, roll)


	# # Yaw, pitch, roll updated from gx, gy, gz only 
	# x[6] = x[6] + gz[i]*dt
	# x[7] = x[7] + gy[i]*dt
	# x[8] = x[8] + gx[i]*dt 
	noKX.append(float(px))
	noKY.append(float(py))
	noKZ.append(float(pz))
	# noKX.append(float(x[0]))
	# noKY.append(float(x[1]))
	# noKZ.append(float(x[2]))

fig = plt.figure(figsize=(15,10))
axplot = fig.gca(projection='3d')
axplot.set_title("Trajectory plotted from ground truth position")
axplot.plot(cumz[:concat_len], cumx[:concat_len], cumy[:concat_len], label = 'ground truth trajectory')
axplot.plot(noKX[:concat_len], noKY[:concat_len], noKZ[:concat_len],label='no kalman filter')
axplot.scatter(noKX[0], noKY[0], noKZ[0], color='r')


axplot.legend()
axplot.set_xlabel('X axis')
axplot.set_ylabel('Y axis')
axplot.set_zlabel('Z axis')
plt.show()





# Kalman filter 
numstates = 9
P = np.eye(numstates)

# P = np.diagflat([[0.01],[0.01],[0.01],[0],[0],[0],[10],[10],[10]])
# P = np.diagflat([[0.01],[0.01],[0.01],[0.1],[0.1],[0.1],[10],[10],[10]])


I = np.eye(numstates)

# jerkmax = 300.0    # m/s3
# pitchrateaccmax=  200.0 *np.pi/180.0 # rad/s2
# rollrateaccmax =  200.0 *np.pi/180.0 # rad/s2
# yawrateaccmax  =  80.0  *np.pi/180.0 # rad/s2

# #make it smaller when you trust IMU more 
# Q = np.diagflat([[(dt * jerkmax)/1000],            # acceleration
#                  [(dt * jerkmax)/1000],            # acceleration
#                  [(dt * jerkmax)/1000],            # acceleration
#             [(dt * yawrateaccmax)/100],           # yawrate
#             [(dt * pitchrateaccmax)/100],         # pitchrate
#             [(dt * rollrateaccmax)/100]])         # rollrate


#make it smaller when you trust IMU more 
# Q = np.diagflat([[100],            # acceleration
# 				 [10],            # acceleration
# 				 [10],            # acceleration
# 			[(10)],           # yawrate
# 			[(10)],         # pitchrate
# 			[(10)]])         # rollrate


# # Make it smaller when you trust GPS more
# R = np.diagflat([[(0.001)],      # x
# 			[(0.001)],           # y
# 			[(0.001)],           # z
# 			[(10)],  # vx
# 			[(10)],
# 			[(1)],
# 			[(10)],  # heading 
# 			[(10)],  # pitch
# 			[(10)]]) # roll




# #make it smaller when you trust IMU more 
Q = np.diagflat([[400],            # acceleration
				 [200],            # acceleration
				 [200],            # acceleration
			[(5)],           # yawrate
			[(2)],         # pitchrate
			[(2)]])         # rollrate


# Make it smaller when you trust GPS more
R = np.diagflat([[(0.01)],      # x
			[(0.01)],           # y
			[(0.01)],           # z
			[(0.1)],  # vx
			[(0.1)],
			[(0.1)],
			[(1.5)],  # heading 
			[(1.5)],  # pitch
			[(1)]]) # roll
# print(P.shape)
# print(JA.shape)
# print(JA.T.shape)
# print(R.shape)
# print(JH.shape)

#
#measurements = np.vstack((hmx, hmy, hmz, vx*0, vy*0, vz*0, hYAW, hPITCH, hROLL))
# print(gtx[0])
# print(len(gtx))

print("length")
print(len(gtx))
print(len(gty))
print(len(gtz))
print(len((np.array(ax)*0)[:concat_len]))
print(len(yawA3_[:concat_len]))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawA3_, pitchA3_, rollA3_))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawOP_, pitchOP_, rollOP_))
measurements = np.vstack((gtx, gty, gtz, (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], yawES_[:concat_len], pitchES_[:concat_len], rollES_[:concat_len]))

P0 = []
P1 = []
P2 = []
P3 = []
P4 = []
P5 = []
P6 = []
P7 = []
P8 = []

GPSpointsX = []
GPSpointsY = []
GPSpointsZ = []

TestYaw = []
TestPitch = []
TestRoll = [] 

X = []
Y = []
Z = []
p_x = 0
p_y = 0
p_z = 0
X.append(p_x)
Y.append(p_y)
Z.append(p_z)

VX = []
VY = []
VZ = []
# TODO: initial vx0 for testing
v_x = 0
v_y = 0
v_z = 0
VX.append(v_x)
VY.append(v_y)
VZ.append(v_z)

AXS = []
AYS = []
AZS = []
AZGS = []

# ya = yaw0
# pi = pitch0
# ro = roll0

Test = []
Test2 = []

GPS_freq = 0

GPSFix = []
fix = 0
for i in range(len(ax)):
	if fix == 0:
		GPSFix.append(True)
		fix = fix+ GPS_freq
	else: 
		fix = fix -1
		GPSFix.append(False)

# GPSFix = []
# for i in range(len(ax)):
# 	GPSFix.append(True)

#state matrix 
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], yaw0, pitch0, roll0]).T
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], 0, 0, 0]).T
x = np.mat([0, 0, 0, 0, 0, 0, 0, 0, 0]).T


for i in range(concat_len): #len(ax)):
#     print(i)
	
######### Prediction STEP starts ####################################################
	x[0] = x[0] + x[3]*dt
	x[1] = x[1] + x[4]*dt
	x[2] = x[2] + x[5]*dt
	
	# x[6]- yaw, x[7] - pitch, x[8]- roll 
	sa = sin(x[6]) # sin (yaw + yawrate)
	ca = cos(x[6]) # cos (yaw + yawrate)
	sb = sin(x[8]) # sin (roll + rollrate)
	cb = cos(x[8]) # cos (roll + rollrate
	sh = sin(x[7]) # sin (pitch + pitchrate)
	ch = cos(x[7]) # cos (pitch + pitchrate)
	
	m00 = ch*ca
	m01 = -ch*sa*cb + sh*sb
	m02 = ch*sa*sb + sh*cb
	m10 = sa
	m11 = ca*cb
	m12 = -ca*sb
	m20 = -sh*ca
	m21 = sh*sa*cb + ch*sb
	m22 = -sh*sa*sb + ch*cb
	
	Axs = (m00*ax[i] + m01*ay[i] + m02*az[i])/9.8
	Ays = (m10*ax[i] + m11*ay[i] + m12*az[i])/9.8
	Azgs = (m20*ax[i] + m21*ay[i] + m22*az[i])/9.8
	Azs = (m20*ax[i] + m21*ay[i] + m22*az[i] - 9.8)/9.8
	
	x[3] = x[3] + Axs*dt
	x[4] = x[4] + Ays*dt
	x[5] = x[5] + Azs*dt

		# Yaw, pitch, roll updated from gx, gy, gz only 
	x[6] = x[6] + gz[i]*dt
	x[7] = x[7] + gy[i]*dt
	x[8] = x[8] + gx[i]*dt 


	X.append(float(x[0]))
	Y.append(float(x[1]))
	Z.append(float(x[2]))
	
	a36 = dt*(-dt*ax[i]*sb*ch - dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	a46 = dt*(dt*ax[i]*cb - dt*ay[i]*ca + dt*az[i]*sb*sa)
	a56 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	a37 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca + dt*sa*ch) + az[i]*(-dt*sh*sb*sa + dt*ch*ca))
	a57 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca -dt*sb*sa*ch))
	
	a38 = dt*(ax[i]*(dt*sh*ca+dt*sb*sa*ch) + az[i]*(-dt*sh*sa + dt*sb*ch*ca))
	a48 = dt*(-dt*ay[i]*sa*cb - dt*az[i]*cb*ca)
	a58 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JA = np.matrix([[1,0,0,dt,0,0,0,0,0],
					[0,1,0,0,dt,0,0,0,0],
					[0,0,1,0,0,dt,0,0,0],
					[0,0,0,1,0,0,a36, a37, a38],
					[0,0,0,0,1,0,a46, 0,   a48],
					[0,0,0,0,0,1,a56, a57, a58],
					[0,0,0,0,0,0,dt,0,0],
					[0,0,0,0,0,0,0,0,dt],
					[0,0,0,0,0,0,0,dt,0]])
	
	g30 = dt*ch*cb
	g40 = dt*sb
	g50 = -dt*sh*cb
	
	g31 = dt*(sh*sa - sb*ch*ca)
	g41 = dt*cb*ca
	g51 = dt*(sh*sb*ca + sa*ch)
	
	g32 = dt*(sh*ca + sb*sa*ch)
	g42 = -dt*sa*cb
	g52 = dt*(-sh*sb*sa + ch*ca)
	
	g33 = dt*(-dt*ax[i]*sb*ch -dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	g43 = dt*(dt*ax[i]*cb - dt*ay[i]*sb*ca + dt*az[i]*sb*sa)
	g53 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	g34 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca+dt*sa*ch)+az[i]*(-dt*sh*sb*sa+dt*ch*ca))
	g54 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca-dt*sb*sa*ch))
	
	g35 = dt*(ay[i]*(dt*sh*ca*dt*sb*sa*ch)+az[i]*(-dt*sh*sa+dt*sb*ch*ca))
	g45 = dt*(-dt*ay[i]*sa*cb -dt*az[i]*cb*ca)
	g55 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JG = np.matrix([[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[g30,g31,g32,g33,g34,g35],
					[g40,g41,g42,g43,0,  g45],
					[g50,g51,g52,g53,g54,g55],
					[0,0,0,dt,0,0],
					[0,0,0,0,0,dt],
					[0,0,0,0,dt,0]])
	
	P = JA*P*JA.T + JG*Q*JG.T
	
	hx = np.matrix([[float(x[0])],
					[float(x[1])],
					[float(x[2])],                    
					[float(x[3])],
					[float(x[4])],
					[float(x[5])],
					[float(x[6])],
					[float(x[7])],
					[float(x[8])]])
	
	if GPSFix[i]:
		print(i)
#         print(float(x[3]))
		#print("GPS")
		# Calculate the Jacobian of the Measurement Function
		# see "Measurement Matrix H"
		JH = np.matrix([[1,0,0,0,0,0,0,0,0],
						[0,1,0,0,0,0,0,0,0],
						[0,0,1,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,1,0,0],
						[0,0,0,0,0,0,0,1,0],
						[0,0,0,0,0,0,0,0,1]
					   ])
		#np.eye(numstates)

		S = JH*P*JH.T + R 

	
		K = (P*JH.T) * np.linalg.inv(np.mat(S,dtype='float'))
		
#         print(K.shape)
#         y = np.mat([[mx[i]-x[0]],[my[i]-x[1]],[mz[i]-x[2]],[0],[0],[0],[YAW[i]-x[6]],[PITCH[i]-x[7]],[ROLL[i]-x[8]]])
#         print(K*y)
#         # Update the estimate via
		z = measurements[:,i].reshape(JH.shape[0],1)
#         print(z)
#         print(z[0][0])
		GPSpointsX.append(float(z[0]))
		GPSpointsY.append(float(z[1]))
		GPSpointsZ.append(float(z[2]))

		y = z - (hx)
		

		x = x + (K*y)


		#print((I - (K*JH)))
		#     # Update the error covariance
		P = (I - (K*JH))*P
		
		
	
######## Prediction STEP ends ####################################################

	P0.append(float(P[0,0]))
	P1.append(float(P[1,1]))
	P2.append(float(P[2,2]))
	P3.append(float(P[3,3]))
	P4.append(float(P[4,4]))
	P5.append(float(P[5,5]))
	P6.append(float(P[6,6]))
	P7.append(float(P[7,7]))
	P8.append(float(P[8,8]))


X_ES = X 
Y_ES = Y
Z_ES = Z 

measurements = np.vstack((gtx, gty, gtz, (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len]))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawA3_, pitchA3_, rollA3_))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawOP_, pitchOP_, rollOP_))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawES_, pitchES_, rollES_))

P0 = []
P1 = []
P2 = []
P3 = []
P4 = []
P5 = []
P6 = []
P7 = []
P8 = []

GPSpointsX = []
GPSpointsY = []
GPSpointsZ = []

TestYaw = []
TestPitch = []
TestRoll = [] 

X = []
Y = []
Z = []
p_x = 0
p_y = 0
p_z = 0
X.append(p_x)
Y.append(p_y)
Z.append(p_z)

VX = []
VY = []
VZ = []
# TODO: initial vx0 for testing
v_x = 0
v_y = 0
v_z = 0
VX.append(v_x)
VY.append(v_y)
VZ.append(v_z)

AXS = []
AYS = []
AZS = []
AZGS = []

# ya = yaw0
# pi = pitch0
# ro = roll0

Test = []
Test2 = []


#state matrix 
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], yaw0, pitch0, roll0]).T
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], 0, 0, 0]).T
x = np.mat([0, 0, 0, 0, 0, 0, 0, 0, 0]).T


for i in range(concat_len): #len(ax)):
#     print(i)
	
######### Prediction STEP starts ####################################################
	x[0] = x[0] + x[3]*dt
	x[1] = x[1] + x[4]*dt
	x[2] = x[2] + x[5]*dt
	
	# x[6]- yaw, x[7] - pitch, x[8]- roll 
	sa = sin(x[6]) # sin (yaw + yawrate)
	ca = cos(x[6]) # cos (yaw + yawrate)
	sb = sin(x[8]) # sin (roll + rollrate)
	cb = cos(x[8]) # cos (roll + rollrate
	sh = sin(x[7]) # sin (pitch + pitchrate)
	ch = cos(x[7]) # cos (pitch + pitchrate)
	
	m00 = ch*ca
	m01 = -ch*sa*cb + sh*sb
	m02 = ch*sa*sb + sh*cb
	m10 = sa
	m11 = ca*cb
	m12 = -ca*sb
	m20 = -sh*ca
	m21 = sh*sa*cb + ch*sb
	m22 = -sh*sa*sb + ch*cb
	
	Axs = (m00*ax[i] + m01*ay[i] + m02*az[i])/9.8
	Ays = (m10*ax[i] + m11*ay[i] + m12*az[i])/9.8
	Azgs = (m20*ax[i] + m21*ay[i] + m22*az[i])/9.8
	Azs = (m20*ax[i] + m21*ay[i] + m22*az[i] - 9.8)/9.8
	
	x[3] = x[3] + Axs*dt
	x[4] = x[4] + Ays*dt
	x[5] = x[5] + Azs*dt

		# Yaw, pitch, roll updated from gx, gy, gz only 
	x[6] = x[6] + gz[i]*dt
	x[7] = x[7] + gy[i]*dt
	x[8] = x[8] + gx[i]*dt 


	X.append(float(x[0]))
	Y.append(float(x[1]))
	Z.append(float(x[2]))
	
	a36 = dt*(-dt*ax[i]*sb*ch - dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	a46 = dt*(dt*ax[i]*cb - dt*ay[i]*ca + dt*az[i]*sb*sa)
	a56 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	a37 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca + dt*sa*ch) + az[i]*(-dt*sh*sb*sa + dt*ch*ca))
	a57 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca -dt*sb*sa*ch))
	
	a38 = dt*(ax[i]*(dt*sh*ca+dt*sb*sa*ch) + az[i]*(-dt*sh*sa + dt*sb*ch*ca))
	a48 = dt*(-dt*ay[i]*sa*cb - dt*az[i]*cb*ca)
	a58 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JA = np.matrix([[1,0,0,dt,0,0,0,0,0],
					[0,1,0,0,dt,0,0,0,0],
					[0,0,1,0,0,dt,0,0,0],
					[0,0,0,1,0,0,a36, a37, a38],
					[0,0,0,0,1,0,a46, 0,   a48],
					[0,0,0,0,0,1,a56, a57, a58],
					[0,0,0,0,0,0,dt,0,0],
					[0,0,0,0,0,0,0,0,dt],
					[0,0,0,0,0,0,0,dt,0]])
	
	g30 = dt*ch*cb
	g40 = dt*sb
	g50 = -dt*sh*cb
	
	g31 = dt*(sh*sa - sb*ch*ca)
	g41 = dt*cb*ca
	g51 = dt*(sh*sb*ca + sa*ch)
	
	g32 = dt*(sh*ca + sb*sa*ch)
	g42 = -dt*sa*cb
	g52 = dt*(-sh*sb*sa + ch*ca)
	
	g33 = dt*(-dt*ax[i]*sb*ch -dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	g43 = dt*(dt*ax[i]*cb - dt*ay[i]*sb*ca + dt*az[i]*sb*sa)
	g53 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	g34 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca+dt*sa*ch)+az[i]*(-dt*sh*sb*sa+dt*ch*ca))
	g54 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca-dt*sb*sa*ch))
	
	g35 = dt*(ay[i]*(dt*sh*ca*dt*sb*sa*ch)+az[i]*(-dt*sh*sa+dt*sb*ch*ca))
	g45 = dt*(-dt*ay[i]*sa*cb -dt*az[i]*cb*ca)
	g55 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JG = np.matrix([[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[g30,g31,g32,g33,g34,g35],
					[g40,g41,g42,g43,0,  g45],
					[g50,g51,g52,g53,g54,g55],
					[0,0,0,dt,0,0],
					[0,0,0,0,0,dt],
					[0,0,0,0,dt,0]])
	
	P = JA*P*JA.T + JG*Q*JG.T
	
	hx = np.matrix([[float(x[0])],
					[float(x[1])],
					[float(x[2])],                    
					[float(x[3])],
					[float(x[4])],
					[float(x[5])],
					[float(x[6])],
					[float(x[7])],
					[float(x[8])]])
	
	if GPSFix[i]:
		print(i)
#         print(float(x[3]))
		#print("GPS")
		# Calculate the Jacobian of the Measurement Function
		# see "Measurement Matrix H"
		JH = np.matrix([[1,0,0,0,0,0,0,0,0],
						[0,1,0,0,0,0,0,0,0],
						[0,0,1,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,1,0,0],
						[0,0,0,0,0,0,0,1,0],
						[0,0,0,0,0,0,0,0,1]
					   ])
		#np.eye(numstates)

		S = JH*P*JH.T + R 

	
		K = (P*JH.T) * np.linalg.inv(np.mat(S,dtype='float'))
		
#         print(K.shape)
#         y = np.mat([[mx[i]-x[0]],[my[i]-x[1]],[mz[i]-x[2]],[0],[0],[0],[YAW[i]-x[6]],[PITCH[i]-x[7]],[ROLL[i]-x[8]]])
#         print(K*y)
#         # Update the estimate via
		z = measurements[:,i].reshape(JH.shape[0],1)
#         print(z)
#         print(z[0][0])
		GPSpointsX.append(float(z[0]))
		GPSpointsY.append(float(z[1]))
		GPSpointsZ.append(float(z[2]))

		y = z - (hx)
		

		x = x + (K*y)


		#print((I - (K*JH)))
		#     # Update the error covariance
		P = (I - (K*JH))*P
		
		
	
######## Prediction STEP ends ####################################################

	P0.append(float(P[0,0]))
	P1.append(float(P[1,1]))
	P2.append(float(P[2,2]))
	P3.append(float(P[3,3]))
	P4.append(float(P[4,4]))
	P5.append(float(P[5,5]))
	P6.append(float(P[6,6]))
	P7.append(float(P[7,7]))
	P8.append(float(P[8,8]))


X_A3 = X
Y_A3 = Y
Z_A3 = Z




# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawA3_, pitchA3_, rollA3_))
measurements = np.vstack((gtx, gty, gtz, (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], (np.array(ax)*0)[:concat_len], yawOP_[:concat_len], pitchOP_[:concat_len], rollOP_[:concat_len]))
# measurements = np.vstack((gtx, gty, gtz, np.array(ax)*0, np.array(ax)*0, np.array(ax)*0, yawES_, pitchES_, rollES_))


P0 = []
P1 = []
P2 = []
P3 = []
P4 = []
P5 = []
P6 = []
P7 = []
P8 = []

GPSpointsX = []
GPSpointsY = []
GPSpointsZ = []

TestYaw = []
TestPitch = []
TestRoll = [] 

X = []
Y = []
Z = []
p_x = 0
p_y = 0
p_z = 0
X.append(p_x)
Y.append(p_y)
Z.append(p_z)

VX = []
VY = []
VZ = []
# TODO: initial vx0 for testing
v_x = 0
v_y = 0
v_z = 0
VX.append(v_x)
VY.append(v_y)
VZ.append(v_z)

AXS = []
AYS = []
AZS = []
AZGS = []

# ya = yaw0
# pi = pitch0
# ro = roll0

Test = []
Test2 = []


# GPSFix = []
# for i in range(len(ax)):
# 	GPSFix.append(True)

#state matrix 
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], yaw0, pitch0, roll0]).T
# x = np.mat([0, 0, 0, vx[0], vy[0], vz[0], 0, 0, 0]).T
x = np.mat([0, 0, 0, 0, 0, 0, 0, 0, 0]).T


for i in range(concat_len): #len(ax)):
#     print(i)
	
######### Prediction STEP starts ####################################################
	x[0] = x[0] + x[3]*dt
	x[1] = x[1] + x[4]*dt
	x[2] = x[2] + x[5]*dt
	
	# x[6]- yaw, x[7] - pitch, x[8]- roll 
	sa = sin(x[6]) # sin (yaw + yawrate)
	ca = cos(x[6]) # cos (yaw + yawrate)
	sb = sin(x[8]) # sin (roll + rollrate)
	cb = cos(x[8]) # cos (roll + rollrate
	sh = sin(x[7]) # sin (pitch + pitchrate)
	ch = cos(x[7]) # cos (pitch + pitchrate)
	
	m00 = ch*ca
	m01 = -ch*sa*cb + sh*sb
	m02 = ch*sa*sb + sh*cb
	m10 = sa
	m11 = ca*cb
	m12 = -ca*sb
	m20 = -sh*ca
	m21 = sh*sa*cb + ch*sb
	m22 = -sh*sa*sb + ch*cb
	
	Axs = (m00*ax[i] + m01*ay[i] + m02*az[i])/9.8
	Ays = (m10*ax[i] + m11*ay[i] + m12*az[i])/9.8
	Azgs = (m20*ax[i] + m21*ay[i] + m22*az[i])/9.8
	Azs = (m20*ax[i] + m21*ay[i] + m22*az[i] - 9.8)/9.8
	
	x[3] = x[3] + Axs*dt
	x[4] = x[4] + Ays*dt
	x[5] = x[5] + Azs*dt

		# Yaw, pitch, roll updated from gx, gy, gz only 
	x[6] = x[6] + gz[i]*dt
	x[7] = x[7] + gy[i]*dt
	x[8] = x[8] + gx[i]*dt 


	X.append(float(x[0]))
	Y.append(float(x[1]))
	Z.append(float(x[2]))
	
	a36 = dt*(-dt*ax[i]*sb*ch - dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	a46 = dt*(dt*ax[i]*cb - dt*ay[i]*ca + dt*az[i]*sb*sa)
	a56 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	a37 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca + dt*sa*ch) + az[i]*(-dt*sh*sb*sa + dt*ch*ca))
	a57 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca -dt*sb*sa*ch))
	
	a38 = dt*(ax[i]*(dt*sh*ca+dt*sb*sa*ch) + az[i]*(-dt*sh*sa + dt*sb*ch*ca))
	a48 = dt*(-dt*ay[i]*sa*cb - dt*az[i]*cb*ca)
	a58 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JA = np.matrix([[1,0,0,dt,0,0,0,0,0],
					[0,1,0,0,dt,0,0,0,0],
					[0,0,1,0,0,dt,0,0,0],
					[0,0,0,1,0,0,a36, a37, a38],
					[0,0,0,0,1,0,a46, 0,   a48],
					[0,0,0,0,0,1,a56, a57, a58],
					[0,0,0,0,0,0,dt,0,0],
					[0,0,0,0,0,0,0,0,dt],
					[0,0,0,0,0,0,0,dt,0]])
	
	g30 = dt*ch*cb
	g40 = dt*sb
	g50 = -dt*sh*cb
	
	g31 = dt*(sh*sa - sb*ch*ca)
	g41 = dt*cb*ca
	g51 = dt*(sh*sb*ca + sa*ch)
	
	g32 = dt*(sh*ca + sb*sa*ch)
	g42 = -dt*sa*cb
	g52 = dt*(-sh*sb*sa + ch*ca)
	
	g33 = dt*(-dt*ax[i]*sb*ch -dt*ay[i]*ch*cb*ca + dt*az[i]*sa*ch*cb)
	g43 = dt*(dt*ax[i]*cb - dt*ay[i]*sb*ca + dt*az[i]*sb*sa)
	g53 = dt*(dt*ax[i]*sh*sb + dt*ay[i]*sh*cb*ca - dt*az[i]*sh*sa*cb)
	
	g34 = dt*(-dt*ax[i]*sh*cb + ay[i]*(dt*sh*sb*ca+dt*sa*ch)+az[i]*(-dt*sh*sb*sa+dt*ch*ca))
	g54 = dt*(-dt*ax[i]*ch*cb + ay[i]*(-dt*sh*sa + dt*sb*ch*ca) + az[i]*(-dt*sh*ca-dt*sb*sa*ch))
	
	g35 = dt*(ay[i]*(dt*sh*ca*dt*sb*sa*ch)+az[i]*(-dt*sh*sa+dt*sb*ch*ca))
	g45 = dt*(-dt*ay[i]*sa*cb -dt*az[i]*cb*ca)
	g55 = dt*(ay[i]*(-dt*sh*sb*sa + dt*ch*ca) + az[i]*(-dt*sh*sb*ca - dt*sa*ch))
	
	JG = np.matrix([[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[0,0,0,0,0,0],
					[g30,g31,g32,g33,g34,g35],
					[g40,g41,g42,g43,0,  g45],
					[g50,g51,g52,g53,g54,g55],
					[0,0,0,dt,0,0],
					[0,0,0,0,0,dt],
					[0,0,0,0,dt,0]])
	
	P = JA*P*JA.T + JG*Q*JG.T
	
	hx = np.matrix([[float(x[0])],
					[float(x[1])],
					[float(x[2])],                    
					[float(x[3])],
					[float(x[4])],
					[float(x[5])],
					[float(x[6])],
					[float(x[7])],
					[float(x[8])]])
	
	if GPSFix[i]:
		print(i)
#         print(float(x[3]))
		#print("GPS")
		# Calculate the Jacobian of the Measurement Function
		# see "Measurement Matrix H"
		JH = np.matrix([[1,0,0,0,0,0,0,0,0],
						[0,1,0,0,0,0,0,0,0],
						[0,0,1,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,0,0,0],
						[0,0,0,0,0,0,1,0,0],
						[0,0,0,0,0,0,0,1,0],
						[0,0,0,0,0,0,0,0,1]
					   ])
		#np.eye(numstates)

		S = JH*P*JH.T + R 

	
		K = (P*JH.T) * np.linalg.inv(np.mat(S,dtype='float'))
		
#         print(K.shape)
#         y = np.mat([[mx[i]-x[0]],[my[i]-x[1]],[mz[i]-x[2]],[0],[0],[0],[YAW[i]-x[6]],[PITCH[i]-x[7]],[ROLL[i]-x[8]]])
#         print(K*y)
#         # Update the estimate via
		z = measurements[:,i].reshape(JH.shape[0],1)
#         print(z)
#         print(z[0][0])
		GPSpointsX.append(float(z[0]))
		GPSpointsY.append(float(z[1]))
		GPSpointsZ.append(float(z[2]))

		y = z - (hx)
		

		x = x + (K*y)


		#print((I - (K*JH)))
		#     # Update the error covariance
		P = (I - (K*JH))*P
		
		
	
######## Prediction STEP ends ####################################################

	P0.append(float(P[0,0]))
	P1.append(float(P[1,1]))
	P2.append(float(P[2,2]))
	P3.append(float(P[3,3]))
	P4.append(float(P[4,4]))
	P5.append(float(P[5,5]))
	P6.append(float(P[6,6]))
	P7.append(float(P[7,7]))
	P8.append(float(P[8,8]))


X_OP = X
Y_OP = Y
Z_OP = Z


fig = plt.figure(figsize=(15,10))
axplot = fig.gca(projection='3d')
axplot.set_title("Trajectory plotted from ground truth position")
axplot.plot(GPSpointsX[:concat_len],GPSpointsY[:concat_len],GPSpointsZ[:concat_len], '.',label='GPS points')
axplot.plot(gtx[:concat_len],gty[:concat_len],gtz[:concat_len], label='ground truth')
# axplot.plot(noKX[:concat_len], noKY[:concat_len], noKZ[:concat_len], label = 'no kalman filter trajectory')
axplot.plot(X_ES,Y_ES,Z_ES ,'.', label='predicted x,y,z with ES alg')
axplot.plot(X_A3,Y_A3,Z_A3 ,  label='predicted x,y,z with A3 algs')
axplot.plot(X_OP,Y_OP,Z_OP, label='predicted x,y,z with OP orientation')


axplot.legend()
axplot.set_xlabel('X axis')
axplot.set_ylabel('Y axis')
axplot.set_zlabel('Z axis')
plt.show()


Error_ES = []
Error_A3 = []
Error_OP = []
Error_NK = []
print("length")
print(len(X_ES))
print(len(X_A3))
print(len(gtx))
# for k in range(len(gtx)):

# 	Error_ES.append(np.sqrt((gtx[k]-X_ES[k])**2 + (gty[k]-Y_ES[k])**2 + (gtz[k] - Z_ES[k])**2))
# 	Error_A3.append(np.sqrt((gtx[k]-X_A3[k])**2 + (gty[k]-Y_A3[k])**2 + (gtz[k] - Z_A3[k])**2))

for k in range(concat_len):
	if k == 0: 
		pass
	else:
		Error_ES.append(np.sqrt((gtx[k-1]-X_ES[k])**2 + (gty[k-1]-Y_ES[k])**2 + (gtz[k-1] - Z_ES[k])**2))
		Error_A3.append(np.sqrt((gtx[k-1]-X_A3[k])**2 + (gty[k-1]-Y_A3[k])**2 + (gtz[k-1] - Z_A3[k])**2))
		Error_OP.append(np.sqrt((gtx[k-1]-X_OP[k])**2 + (gty[k-1]-Y_OP[k])**2 + (gtz[k-1] - Z_OP[k])**2))
		Error_NK.append(np.sqrt((gtx[k-1]-noKX[k])**2 + (gty[k-1]-noKY[k])**2 + (gtz[k-1] - noKZ[k])**2))



plt.plot(Error_ES, label = 'ES')
plt.plot(Error_A3, label='A3')
plt.plot(Error_OP, label='OP')
# plt.plot(Error_NK, label='no kalman filter')

plt.legend()
plt.show()



m = concat_len
fig = plt.figure(figsize=(16,9))
plt.semilogy(range(m),P6, label='$x$')
plt.step(range(m),P7, label='$y$')
plt.step(range(m),P8, label='$z$')
# plt.step(range(m),P3, label='$vx$')
# plt.step(range(m),P4, label='$vy$')
# plt.step(range(m),P5, label='$vz$')
# plt.step(range(m),P6, label='$\phi$')
# plt.step(range(m),P7, label='$\Theta$')
# plt.step(range(m),P7, label='$\psi$')

plt.xlabel('Filter Step [k]')
plt.ylabel('')
plt.xlim(0,m)
plt.title('Uncertainty (Elements from Matrix $P$)')
#plt.legend(loc='best',prop={'size':22})
plt.legend(bbox_to_anchor=(0., 0.91, 1., .06), loc=3,
	   ncol=9, mode="expand", borderaxespad=0.,prop={'size':22})
# plt.savefig('Covariance-Matrix-Verlauf.eps', bbox_inches='tight')
plt.show()





