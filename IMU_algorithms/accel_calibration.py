# accel calibration

# Sum all 
# divide so get average 
# remove gravity from z-axis 
	# if az > g , az -= g 
	# else, az +=g



from modules.mpulib import computeheading, attitudefromCompassGravity
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
from modules.a3muse import quatNormalized, IntegrationRK4, EulerToQuat, AccMagOrientation, headingfromMag, QuatToEuler, angle_between, QuatToRotMat, AxisAngleToRotMat, RotMatToQuat
from math import atan2, atan
from numpy.linalg import inv
from numpy import linalg as LA
import matplotlib.pyplot as plt
# import euclid


######## Calibrateds accelerometer and prints the calibration values. First part records accelerometer data at different positions. Second part displays the plot for calibration and its scale values. #################

###### This part records accelerometer values to calibrate. Hold accelerometer at different positions at each part. ########################


# filename = open('accel_values_for_calibration.txt','w')
#
# i = 0
#
# import serial 
# ser = serial.Serial('/dev/tty.usbmodem14411')
# ser.baudrate = 115200
# ser.timeout = 3



# # reading: time, ax, ay, az, heading
# while i < 50 : 
# 	print("cal-1")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1


# i = 0 

# while i < 60 : 
# 	print("cal-1 ends")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]

# 	i = i + 1


# i = 0 

# while i < 50 : 
# 	print("cal-2")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1
# i = 0 

# while i < 60 : 
# 	print("cal-2 ends")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]

# 	i = i + 1


# i = 0 

# while i < 50 : 
# 	print("cal-3")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1

# i = 0 

# while i < 60 : 
# 	print("cal-3 ends")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]

# 	i = i + 1


# i = 0 



# while i < 50 : 
# 	print("cal-4")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1

# i = 0 

# while i < 60 : 
# 	print("cal-4 ends")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]

# 	i = i + 1



# i = 0 

# while i < 50 : 
# 	print("cal-5")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1


# i = 0 

# while i < 60 : 
# 	print("cal-5 ends")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]

# 	i = i + 1




# i = 0 

# while i < 50 : 
# 	print("cal-6")
# 	reading = ser.readline()
# 	#print(reading)
# 	sp = str(reading).split(',')
# 	# print(sp)

# 	time = float(sp[0][2:].strip())

# 	ax = float(sp[1].strip())
# 	ay = float(sp[2].strip())
# 	az = float(sp[3].strip())

# 	accel = [ax, ay, az]
# 	print(i)
# 	print(accel)
# 	print(ax, ay, az, file = filename)
# 	i = i + 1


# filename.close()


################## # Displays plot for the acceleration calibration  ############################

#filename = open('accel_values_for_calibration.txt','r')
#
#ax = []
#ay = []
#az = []


# for i in filename: 
# 	sp = i.split()
# 	mx.append(float(sp[0]))
# 	my.append(float(sp[1]))
# 	mz.append(float(sp[2]))
# 	# print(sp[0])

# filename.close()


# offset_mx = (max(mx) + min(mx)) / 2
# offset_my = (max(my) + min(my)) / 2
# offset_mz = (max(mz) + min(mz)) / 2


# print(offset_mx)
# print(offset_my)
# print(offset_mz)
# cmx = []
# cmy = []
# cmz = []

# # print(mx)
# print(len(mx))
# for k in range(len(mx)):
#     cmx.append(mx[k] - offset_mx)
#     cmy.append(my[k] - offset_my)
#     cmz.append(mz[k] - offset_mz)


# avg_delta_mx = (max(cmx) - min(cmx)) / 2
# avg_delta_my = (max(cmy) - min(cmy)) / 2
# avg_delta_mz = (max(cmz) - min(cmz)) / 2

# avg_delta = (avg_delta_mx + avg_delta_my + avg_delta_mz) / 3

# scale_mx = avg_delta / avg_delta_mx
# scale_my = avg_delta / avg_delta_my
# scale_mz = avg_delta / avg_delta_mz

# print("scale_mx: ", scale_mx)
# print("scale_my: ", scale_my)
# print("scale_mz: ", scale_mz)

# smx = []
# smy = []
# smz = []

# for k in range(len(cmx)):
#     smx.append(cmx[k]*scale_mx)
#     smy.append(cmy[k]*scale_my)
#     smz.append(cmz[k]*scale_mz)



# plt.plot(mx, my, '.')
# plt.plot(my, mz, '.')
# plt.plot(mx, mz, '.')
# plt.title("mxyz")
# plt.show()

# plt.plot(cmx, cmy, '.')
# plt.plot(cmy, cmz, '.')
# plt.plot(cmx, cmz, '.')
# plt.title("cmxyz")
# plt.show()


# plt.plot(smx, smy, '.')
# plt.plot(smy, smz, '.')
# plt.plot(smx, smz, '.')
# plt.title("smxyz")

# plt.show()


