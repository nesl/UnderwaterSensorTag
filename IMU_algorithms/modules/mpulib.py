import math 
from math import atan2, cos, sin, asin,atan
import numpy as np
from modules.a3muse import EulerToQuat
from numpy import linalg as LA
from modules.quaternion import QuaternionClass


def MPU9250_computeEuler(qw, qx, qy, qz):
	dqw = qw
	dqx = qx 
	dqy = qy
	dqz = qz 

	ysqr = dqy**2 
	t0 = -2.0 * (ysqr + dqz * dqz) + 1.0
	t1 = +2.0 * (dqx * dqy - dqw * dqz)
	t2 = -2.0 * (dqx * dqz + dqw * dqy)
	t3 = +2.0 * (dqy * dqz - dqw * dqx)
	t4 = -2.0 * (dqx * dqx + ysqr) + 1.0
  
	if t2 > 1.0: 
		t2 = 1.0 
	if t2 < -1.0: 
		t2 = -1.0
  
	pitch = asin(t2) * 2
	roll = atan2(t3, t4)
	yaw = atan2(t1, t0)

	# pitch *= 180.0 / math.pi
	# roll *= 180.0/math.pi
	# yaw *= 180.0 /math.pi
	# if pitch < 0.0: 
	# 	pitch = 360.0 + pitch 
	# if roll < 0.0: 
	# 	roll = 360.0 + roll 
	# if yaw < 0.0: 
	# 	yaw = 360.0 + yaw 

	return yaw, pitch, roll





	
def quaternion_to_euler_angle(w, x, y, z):
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	X = math.atan2(t0, t1)
	# X = math.degrees(math.atan2(t0, t1))

	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.asin(t2)
	# Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	Z = math.atan2(t3,t4)
	# Z = math.degrees(math.atan2(t3, t4))
	
	return Y, Z, X

def computeheading(mx,my):
	if (my == 0):
		if mx < 0: 
			heading = math.pi
		else: 
			heading = 0 
	else: 
		heading = math.atan2(mx,my)

	# if heading > math.pi:
	# 	heading -= (2*math.pi)
	# elif heading < -math.pi: 
	# 	heading += 2*math.pi
	# elif heading < 0:
	# 	heading += 2*math.pi

	# heading *= 180.0/math.pi

	return heading

def computerollandpitch(accel, mag):
	ax = accel[0]
	ay = accel[1]
	az = accel[2]

	mx = mag[0]
	my = mag[1]
	mz = mag[2]

	roll = atan2(ay,az)
	pitch = atan(-ax/(ay*sin(roll)+az*cos(roll)))

	numerator = mz*sin(roll) - my*cos(roll)
	denominator = mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll)

	yaw = atan2(numerator, denominator)

	return roll, pitch, yaw






def attitudefromCompassGravity(accel, mag): 

	accel = np.array(accel)/LA.norm(accel)
	mag = np.array(mag)/LA.norm(mag)
	# print("accel: " , accel)
	ax = accel[0]
	ay = accel[1]
	az = accel[2]
	g = np.sqrt(ax**2 + ay**2 + az**2)
	# print("g: ", g) 
	# print("ax: ", ax)
	a = -ax/g
	b = -ay/g
	c = -az/g

	# print("a: ", a)
	# print("b: ", b)

	mx = mag[0]
	my = mag[1]

	heading = computeheading(mx,my)*math.pi/180

	ch = cos(heading)
	sh = sin(heading)

	# print("heading: ", heading)
	# print(ch)
	# print(sh)

	# print("ach-bsh:", (a*ch-b*sh))
	# print("ach-ab: ",(a*ch-a*b))
	K3 = ((a*ch-a*b)*(a*ch-b*sh))**2
	# print("K3: ", K3)
	K1 = K3/((a**2)*(a*ch - b*sh)**2)
	# print("den: ", ((a**2)*(a*ch - b*sh)**2))
	# print("K1: ", K1)
	K2 = ((b**2 - c*ch)*(a*ch-a*b))**2

	K = K1 + K2 + K3 
	SK = np.sqrt(K)

	m00 = ((b**2 - c*ch)*SK)/(a*ch - b*sh)
	m10 = (a*c*SK)/(a*ch-a*b)
	m20 = SK
	m01 = sh
	m11 = ch 
	m21 = b
	m02 = a
	m12 = b
	m22 = c

	R = np.matrix([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])
	return R 


def RP_calculate(accel):
	ax = accel[0]
	ay = accel[1]
	az = accel[2]

	g = 9.8
	gx = ax/g
	gy = ay/g
	gz = az/g

	roll = atan2(gy, gz)
	pitch = atan2(-gx, np.sqrt(gy**2 + gz**2 ))
	return roll, pitch




def MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, quat):

	beta = 0.01
	deltat = 0.1
	q1 = quat[0]
	q2 = quat[1]
	q3 = quat[2]
	q4 = quat[3]



	_2q1 = 2.0 * q1
	_2q2 = 2.0 * q2
	_2q3 = 2.0 * q3
	_2q4 = 2.0 * q4
	_2q1q3 = 2.0 * q1 * q3
	_2q3q4 = 2.0 * q3 * q4

	q1q1 = q1 * q1
	q1q2 = q1 * q2
	q1q3 = q1 * q3
	q1q4 = q1 * q4
	q2q2 = q2 * q2
	q2q3 = q2 * q3
	q2q4 = q2 * q4
	q3q3 = q3 * q3
	q3q4 = q3 * q4
	q4q4 = q4 * q4



	norm = np.sqrt(ax**2 + ay**2 + az**2)
	if norm == 0.0:
		print("wrong ")
	norm = 1.0/norm 
	ax *= norm
	ay *= norm
	az *= norm


	norm = np.sqrt(mx**2 + my**2 + mz**2)
	if norm == 0.0:
		print("wrong")
	norm = 1.0/norm
	mx *= norm
	my *= norm
	mz *= norm 

	_2q1mx = 2.0 * q1 * mx
	_2q1my = 2.0 * q1 * my
	_2q1mz = 2.0 * q1 * mz
	_2q2mx = 2.0 * q2 * mx



	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
	_2bx = np.sqrt(hx * hx + hy * hy)
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
	_4bx = 2.0 * _2bx
	_4bz = 2.0 * _2bz


	s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
	s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
	s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
	s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)

	norm = np.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
	norm = 1.0/norm
	s1 *= norm
	s2 *= norm
	s3 *= norm
	s4 *= norm


	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

	q1 += qDot1 * deltat
	q2 += qDot2 * deltat
	q3 += qDot3 * deltat
	q4 += qDot4 * deltat
	norm = np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
	norm = 1.0/norm
	# q = QuaternionClass(1,0,0,0)
	# q[0] = q1 * norm
	# q[1] = q2 * norm
	# q[2] = q3 * norm
	# q[3] = q4 * norm

	quat = QuaternionClass(q1 * norm, q2 * norm, q3 * norm,  q4 * norm)

	return quat 	


def Euler2Quat(yaw, pitch, roll):
	cy = cos(yaw*0.5)
	sy = sin(yaw*0.5)
	cr = cos(roll*0.5)
	sr = sin(roll*0.5)
	cp = cos(pitch*0.5)
	sp = sin(pitch*0.5)

	qw = cy * cr * cp + sy * sr * sp
	qx = cy * sr * cp - sy * cr * sp
	qy = cy * sr * cp - sy * cr * sp
	qz = sy * cr * cp - cy * sr * sp

	q = QuaternionClass(qw, qx, qy, qz)

	return q 






# // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
# // measured ones.
# void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
# {
# float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
# float norm;
# float hx, hy, bx, bz;
# float vx, vy, vz, wx, wy, wz;
# float ex, ey, ez;
# float pa, pb, pc;
# // Auxiliary variables to avoid repeated arithmetic
# float q1q1 = q1 * q1;
# float q1q2 = q1 * q2;
# float q1q3 = q1 * q3;
# float q1q4 = q1 * q4;
# float q2q2 = q2 * q2;
# float q2q3 = q2 * q3;
# float q2q4 = q2 * q4;
# float q3q3 = q3 * q3;
# float q3q4 = q3 * q4;
# float q4q4 = q4 * q4;
# // Normalise accelerometer measurement
# norm = sqrt(ax * ax + ay * ay + az * az);
# if (norm == 0.0f) return; // handle NaN
# norm = 1.0f / norm;        // use reciprocal for division
# ax *= norm;
# ay *= norm;
# az *= norm;
# // Normalise magnetometer measurement
# norm = sqrt(mx * mx + my * my + mz * mz);
# if (norm == 0.0f) return; // handle NaN
# norm = 1.0f / norm;        // use reciprocal for division
# mx *= norm;
# my *= norm;
# mz *= norm;
# // Reference direction of Earth’s magnetic field
# hx = 2.0f * mx * (0.5f – q3q3 – q4q4) + 2.0f * my * (q2q3 – q1q4) + 2.0f * mz * (q2q4 + q1q3);
# hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f – q2q2 – q4q4) + 2.0f * mz * (q3q4 – q1q2);
# bx = sqrt((hx * hx) + (hy * hy));
# bz = 2.0f * mx * (q2q4 – q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f – q2q2 – q3q3);
# // Estimated direction of gravity and magnetic field
# vx = 2.0f * (q2q4 – q1q3);
# vy = 2.0f * (q1q2 + q3q4);
# vz = q1q1 – q2q2 – q3q3 + q4q4;
# wx = 2.0f * bx * (0.5f – q3q3 – q4q4) + 2.0f * bz * (q2q4 – q1q3);
# wy = 2.0f * bx * (q2q3 – q1q4) + 2.0f * bz * (q1q2 + q3q4);
# wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f – q2q2 – q3q3);
# // Error is cross product between estimated direction and measured direction of gravity
# ex = (ay * vz – az * vy) + (my * wz – mz * wy);
# ey = (az * vx – ax * vz) + (mz * wx – mx * wz);
# ez = (ax * vy – ay * vx) + (mx * wy – my * wx);
# if (Ki > 0.0f)
# {
# eInt[0] += ex;      // accumulate integral error
# eInt[1] += ey;
# eInt[2] += ez;
# }
# else
# {
# eInt[0] = 0.0f;     // prevent integral wind up
# eInt[1] = 0.0f;
# eInt[2] = 0.0f;
# }
# // Apply feedback terms
# gx = gx + Kp * ex + Ki * eInt[0];
# gy = gy + Kp * ey + Ki * eInt[1];
# gz = gz + Kp * ez + Ki * eInt[2];
# // Integrate rate of change of quaternion
# pa = q2;
# pb = q3;
# pc = q4;
# q1 = q1 + (-q2 * gx – q3 * gy – q4 * gz) * (0.5f * deltat);
# q2 = pa + (q1 * gx + pb * gz – pc * gy) * (0.5f * deltat);
# q3 = pb + (q1 * gy – pa * gz + pc * gx) * (0.5f * deltat);
# q4 = pc + (q1 * gz + pa * gy – pb * gx) * (0.5f * deltat);
# // Normalise quaternion
# norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
# norm = 1.0f / norm;
# q[0] = q1 * norm;
# q[1] = q2 * norm;
# q[2] = q3 * norm;
# q[3] = q4 * norm;
# }  