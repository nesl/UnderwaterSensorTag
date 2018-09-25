import numpy as np
import math
from math import sin, cos, atan2, atan, asin
from quaternion import QuaternionClass


def IntegrationRK4(omega0, omega1, quat, dt):

        omega01 = 0.5*(np.array(omega0)+ np.array(omega1))

        omega_skew = computeOmegaskew(omega0)
        quatT = np.mat([[quat[0]],[quat[1]],[quat[2]],[quat[3]]])

        K1 = 0.5*omega_skew*quatT

        tmp_q = quatT + dt*0.5*K1
        omega_skew = computeOmegaskew(omega01)
        K2 = 0.5*omega_skew*tmp_q

        tmp_q = quatT + dt*0.5*K2
        K3 = 0.5*omega_skew*tmp_q

        tmp_q = quatT + dt*K3

        omega_skew = computeOmegaskew(omega1)
        K4 = 0.5*omega_skew*tmp_q

        qdot = np.transpose(dt*(1/6)*(K1 + 2*K2 + 2*K3 + K4))
        quat= quat + QuaternionClass(np.array(qdot)[0][0], np.array(qdot)[0][1], np.array(qdot)[0][2], np.array(qdot)[0][3])
        return quat

def AccMagOrientation(accel, mag):
# source: https://www.nxp.com/files-static/sensors/doc/app_note/AN4248.pdf


        Gpx = accel[0]
        Gpy = accel[1]
        Gpz = accel[2]

        #roll and pitch angle from accelerometer
        roll = atan2(Gpy, Gpz)
        pitch = atan(-Gpx/(Gpy*sin(roll) + Gpz*cos(roll)))

        Bpx = mag[0]
        Bpy = mag[1]
        Bpz = mag[2]

        Bfx = Bpx*cos(pitch) + Bpy*sin(pitch)*sin(roll) + Bpz*sin(pitch)*cos(roll)
        Bfy = Bpy*cos(roll) - Bpz*sin(roll)

        yaw = atan2(-Bfy, Bfx)

        q = EulerToQuat(yaw, pitch, roll)
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]
        quat = QuaternionClass(qw, qx, qy, qz)

        return (yaw, pitch, roll, quat)



def computeOmegaskew(w):
    wx = w[0]
    wy = w[1]
    wz = w[2]

    omega = np.mat([[0, -wx, -wy, -wz],[wx, 0 , wz, -wy],[wy, -wz, 0, wx],[wz, wy, -wx, 0]])
    return omega


def QuatToRotMat(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]

    R00 = q1**2 - q2**2 -q3**2 +q4**2
    R01 = 2*(q1*q2 - q4*q3)
    R02 = 2*(q1*q3 + q4*q2)
    R10 = 2*(q1*q2 + q4*q3)
    R11 = -q1**2 + q2**2 - q3**2 + q4**2
    R12 = 2*(q2*q3 - q4*q1)
    R20 = 2*(q3*q1 - q4*q2)
    R21 = 2*(q3*q2 + q4*q1)
    R22 = 2*q1**2 - q2**2 + q3**2 + q4**2

    R = np.array([[R00, R01, R02],[R10, R11, R12],[R20, R21, R22]])
    return R

def RotMatToQuat(R):
    m00 = R[0][0]
    m01 = R[0][1]
    m02 = R[0][2]
    m10 = R[1][0]
    m11 = R[1][1]
    m12 = R[1][2]
    m20 = R[2][0]
    m21 = R[2][1]
    m22 = R[2][2]

    w = np.sqrt(1 + m00 + m11 + m22)/2.0
    w4 = 4.0 * w
    x = (m21 - m12)/w4
    y = (m02 - m20)/w4
    z = (m10 - m01)/w4

    q = [w, x, y, z]
    return q

def EulerToRotMat(yaw, pitch, roll):
    attitude = yaw # rotate about z 
    bank = roll # rotate about x 
    heading = pitch # rotate about y 
    sa = sin(attitude)
    ca = cos(attitude)
    sb = sin(bank)
    cb = cos(bank)
    sh = sin(heading)
    ch = cos(heading)

    m00 = ch*ca
    m01 = -ch*sa*cb + sh*sb
    m02 = ch*sa*sb + sh*cb
    m10 = sa
    m11 = ca*cb
    m12 = -ca*sb
    m20 = -sh*ca
    m21 = sh*sa*cb + ch*sb
    m22 = -sh*sa*sb + ch*cb

    R = np.array([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])
    return R 

def EulerToQuat(yaw, pitch, roll):
    R = EulerToRotMat(yaw, pitch, roll)
    q = RotMatToQuat(R)
    return q 


def QuatToEuler(q):

    # print("q before")
    # print(q[0], q[1], q[2], q[3])

    q = quatNormalized(q)
    # print("q after")
    # print(q[0], q[1], q[2], q[3])

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    test = qx*qy + qz*qw 
    if test > 0.499: 
        pitch = 2*atan2(qx, qw)
        yaw = math.pi/2.0
        roll = 0
        return yaw, pitch, roll
    if test < -0.499: 
        pitch = -2 * atan2(qx, qw)
        yaw = - math.pi/2.0
        roll = 0
        return yaw, pitch, roll

    sqx = qx*qx
    sqy = qy*qy
    sqz = qz*qz
    pitch = atan2(2*qy*qw - 2*qx*qz, 1-2*sqy - 2*sqz)
    yaw = asin(2*test)
    roll = atan2(2*qx*qw-2*qy*qz, 1-2*sqx-2*sqz)

    return yaw, pitch, roll

def headingfromMag(mag):
    mx = mag[0]
    my = mag[1]
    mz = mag[2]

    if my > 0 : 
        headingM = 90 - atan(mx/my)*180/math.pi
    if my < 0 : 
        headingM = 270 - atan(mx/my)*180/math.pi
    if my == 0:
        if mx < 0:
            headingM = 180.0
        else: 
            headingM = 0.0

    return headingM


def quatNormalized(q):


    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    mag_sqrd = qw*qw + qx*qx + qy*qy + qz*qz
    norm = np.sqrt(mag_sqrd)

    qw = qw/norm
    qx = qx/norm
    qy = qy/norm
    qz = qz/norm

    q = [qw, qx, qy, qz]
    return q



