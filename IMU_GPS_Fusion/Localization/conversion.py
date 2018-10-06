# author: eslee

import math 
import numpy as np

# accelx, accely, accelz = 0.0, 0.0, 0.0 # unit: m/s^2
# gyrox, gyroy, gyroz = 0.0, 0.0, 0.0 #/57.3 if unit in radian per second
# # unit: dps 
# magx, magy, magz = 0.0, 0.0, 0.0 # unit: uT

# IMUdata = [accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz]

def IMUtoEulersangle(IMUdata):
    # similar to: https://gist.github.com/shoebahmedadeel/0d8ca4eaa65664492cf1db2ab3a9e572

    accelx, accely, accelz = IMUdata[0], IMUdata[1], IMUdata[2]
    gyrox, gyroy, gyroz = IMUdata[3], IMUdata[4], IMUdata[5]
    magx, magy, magz = IMUdata[6], IMUdata[7], IMUdata[8]
    
    pitch = np.arctan2(accely, np.sqrt((accelx*accelx)+(accelz*accelz)))
    roll = np.arctan2(-accelx, np.sqrt((accely*accely)+(accelz*accelz)))

    Yh = (magy * math.cos(roll)) - (magz * math.sin(roll))
    Xh = (magx * math.cos(pitch))+(magy * math.sin(roll)*math.sin(pitch)) + (magz * math.cos(roll) * math.sin(pitch))
    yaw =  np.arctan2(Yh, Xh)

    roll = roll*57.3
    pitch = pitch*57.3
    yaw = yaw*57.3
    
    return yaw, pitch, roll


# yaw = 0
# pitch = 0 
# roll = 90

def eulerstoQuaternion(yaw,pitch,roll):
    yaw = yaw/57.3
    pitch = pitch/57.3
    roll = roll/57.3

    c1 = math.cos(pitch/2)
    c2 = math.cos(yaw/2)
    c3 = math.cos(roll/2)
    s1 = math.sin(pitch/2)
    s2 = math.sin(yaw/2)
    s3 = math.sin(roll/2)

    w = c1*c2*c3 - s1*s2*s3 
    x = s1*s2*c3 + c1*c2*s3
    y = s1*c2*c3 + c1*s2*s3 
    z = c1*s2*c3 - s1*c2*s3 
    
    return [w,x,y,z]


def quaterniontorotmatrix(quaternion):
    qw,qx,qy,qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

    sqw = qw*qw
    sqx = qx*qx
    sqy = qy*qy
    sqz = qz*qz

    invs = 1/(sqx + sqy + sqz + sqw)
    m00 = (sqx - sqy - sqz + sqw)*invs
    m11 = (-sqx + sqy - sqz + sqw)*invs
    m22 = (-sqx - sqy + sqz + sqw)*invs 


    tmp1 = qx*qy
    tmp2 = qz*qw;
    m10 = 2.0 * (tmp1 + tmp2)*invs 
    m01 = 2.0 * (tmp1 - tmp2)*invs 

    tmp1 = qx*qz
    tmp2 = qy*qw
    m20 = 2.0 * (tmp1 - tmp2)*invs 
    m02 = 2.0 * (tmp1 + tmp2)*invs 
    tmp1 = qy*qz
    tmp2 = qx*qw
    m21 = 2.0 * (tmp1 + tmp2)*invs 
    m12 = 2.0 * (tmp1 - tmp2)*invs   

    rot_matrix = np.matrix([[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]])
    
    return rot_matrix

