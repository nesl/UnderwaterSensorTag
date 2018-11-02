from mpulib import computeheading, attitudefromCompassGravity
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
from a3muse import quatNormalized, IntegrationRK4, EulerToQuat, AccMagOrientation, headingfromMag, QuatToEuler, angle_between, QuatToRotMat, AxisAngleToRotMat, RotMatToQuat
from math import atan2, atan
from numpy.linalg import inv
from numpy import linalg as LA
import matplotlib.pyplot as plt
# import euclid

filename = open('mag_calib.txt','w')

i = 0

import serial 
ser = serial.Serial('/dev/tty.usbmodem14611')
ser.baudrate = 115200
ser.timeout = 3



# reading: time, mx, my, mz, heading
while i < 2000 : 
	reading = ser.readline()
	print(reading)
	#print(reading)
	sp = str(reading).split(',')
	# print(sp)

	time = float(sp[0][2:].strip())

	mx = float(sp[7].strip())
	my = float(sp[8].strip())
	mz = float(sp[9].strip())

	mag = [mx, my, mz]
	print(i)
	print(mag)
	print(mx, my, mz, file = filename)
	i = i + 1

print("calibration done")
filename.close()

filename = open('mag_calib.txt','r')

mx = []
my = []
mz = []

for i in filename: 
	sp = i.split()
	mx.append(float(sp[0]))
	my.append(float(sp[1]))
	mz.append(float(sp[2]))
	# print(sp[0])

filename.close()


offset_mx = (max(mx) + min(mx)) / 2
offset_my = (max(my) + min(my)) / 2
offset_mz = (max(mz) + min(mz)) / 2
print("offset_mx = ",offset_mx)
print("offset_my = ", offset_my)
print("offset_mz = ", offset_mz)

cmx = []
cmy = []
cmz = []

# print(mx)
# print(len(mx))
for k in range(len(mx)):
    cmx.append(mx[k] - offset_mx)
    cmy.append(my[k] - offset_my)
    cmz.append(mz[k] - offset_mz)


avg_delta_mx = (max(cmx) - min(cmx)) / 2
avg_delta_my = (max(cmy) - min(cmy)) / 2
avg_delta_mz = (max(cmz) - min(cmz)) / 2

avg_delta = (avg_delta_mx + avg_delta_my + avg_delta_mz) / 3

scale_mx = avg_delta / avg_delta_mx
scale_my = avg_delta / avg_delta_my
scale_mz = avg_delta / avg_delta_mz

print("scale_mx = ", scale_mx)
print("scale_my = ", scale_my)
print("scale_mz = ", scale_mz)

smx = []
smy = []
smz = []

for k in range(len(cmx)):
    smx.append(cmx[k]*scale_mx)
    smy.append(cmy[k]*scale_my)
    smz.append(cmz[k]*scale_mz)



plt.plot(mx, my, '.')
plt.plot(my, mz, '.')
plt.plot(mx, mz, '.')
plt.title("mxyz")
plt.show()

plt.plot(cmx, cmy, '.')
plt.plot(cmy, cmz, '.')
plt.plot(cmx, cmz, '.')
plt.title("cmxyz")
plt.show()


plt.plot(smx, smy, '.')
plt.plot(smy, smz, '.')
plt.plot(smx, smz, '.')
plt.title("smxyz")

plt.show()


