# Author: eunsunlee
import numpy as np
import math
from math import sin, cos, atan2, atan, asin
from modules.quaternion import QuaternionClass
from numpy import linalg as LA


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
        quat = qnormalized(quat)
        return quat

def AccMag2Euler(accel, mag):
    # accel = np.array(accel)/LA.norm(accel)
    # mag = np.array(mag)/LA.norm(mag)

    g = np.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
    # accel = np.array(accel)/g

    yaw = atan2(mag[1], mag[0])
    roll = atan2(accel[1], accel[2])
    pitch = atan(-accel[0]/(accel[1]*sin(roll) + accel[2]*cos(roll)))
   # pitch = atan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
    

    quat = EulerToQuat(yaw, pitch, roll)

    q = QuaternionClass(quat[0], quat[1], quat[2], quat[3])
    q = qnormalized(q)

    return yaw, pitch, roll, quat

def androidAccMag2Euler(accel,mag):
    Ax = accel[0]
    Ay = accel[1]
    Az = accel[2]
    Ex = mag[0]
    Ey = mag[1]
    Ez = mag[2]

    Hx = Ey*Az - Ez*Ay
    Hy = Ez*Ax - Ex*Az
    Hz = Ex*Ay - Ey*Ax
    normH = np.sqrt(Hx*Hx + Hy*Hy + Hz*Hz)

    if normH < 0.1: 
        print("free fall")

    invH = 1.0/normH
    Hx *= invH 
    Hy *= invH
    Hz *= invH 
    invA = 1.0/np.sqrt(Ax**2 + Ay**2 + Az**2)
    Ax *= invA
    Ay *= invA
    Az *= invA
    Mx = Ay*Hz - Az*Hy
    My = Az*Hx - Ax*Hz
    Mz = Ax*Hy - Ay*Hx

    m00 = Hx 
    m01 = Hy 
    m02 = Hz 
    m10 = Mx 
    m11 = My 
    m12 = Mz 
    m20 = Ax 
    m21 = Ay 
    m22 = Az 

    R = np.array([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])

    yaw = atan2(Hy, My)
    pitch = math.asin(-Ay)
    roll = atan2(-Ax, Az)

    q = EulerToQuat(yaw, pitch, roll)

    # q = RotMatToQuat(R)    
    q = QuaternionClass(q[0], q[1], q[2], q[3])
    # q = qnormalized(q)

    # yaw, pitch, roll = QuatToEuler(q)

    return yaw, pitch, roll, q






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


def angle_axis2quat(theta, vector, is_normalized=False):

    # https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py

    ''' Quaternion for rotation of angle `theta` around `vector`

    Parameters
    ----------
    theta : scalar
       angle of rotation
    vector : 3 element sequence
       vector specifying axis for rotation.
    is_normalized : bool, optional
       True if vector is already normalized (has norm of 1).  Default
       False

    Returns
    -------
    quat : 4 element sequence of symbols
       quaternion giving specified rotation

    Examples
    --------
    >>> q = angle_axis2quat(np.pi, [1, 0, 0])
    >>> np.allclose(q, [0, 1, 0,  0])
    True

    Notes
    -----
    Formula from http://mathworld.wolfram.com/EulerParameters.html
    '''
    vector = np.array(vector)
    if not is_normalized:
        # Cannot divide in-place because input vector may be integer type,
        # whereas output will be float type; this may raise an error in versions
        # of numpy > 1.6.1
        vector = vector / math.sqrt(np.dot(vector, vector))
    t2 = theta / 2.0
    st2 = math.sin(t2)
    return np.concatenate(([math.cos(t2)],
                           vector * st2))


def computeOmegaskew(w):
    wx = w[0]
    wy = w[1]
    wz = w[2]

    omega = np.mat([[0, -wx, -wy, -wz],[wx, 0 , wz, -wy],[wy, -wz, 0, wx],[wz, wy, -wx, 0]])
    return omega

_EPS = np.finfo(float).eps * 4.0

# def quaternion_matrix(quaternion):
#     """Return homogeneous rotation matrix from quaternion.

#     >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
#     >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
#     True
#     >>> M = quaternion_matrix([1, 0, 0, 0])
#     >>> numpy.allclose(M, numpy.identity(4))
#     True
#     >>> M = quaternion_matrix([0, 1, 0, 0])
#     >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
#     True

#     """
#     q = np.array(quaternion, dtype=np.float64, copy=True)
#     n = np.dot(q, q)
#     if n < _EPS:
#         return np.identity(4)
#     q *= math.sqrt(2.0 / n)
#     q = np.outer(q, q)
#     return np.array([
#         [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
#         [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
#         [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
#         [                0.0,                 0.0,                 0.0, 1.0]])


# def quaternion_from_matrix(matrix, isprecise=False):
#     """Return quaternion from rotation matrix.

#     If isprecise is True, the input matrix is assumed to be a precise rotation
#     matrix and a faster algorithm is used.

#     >>> q = quaternion_from_matrix(numpy.identity(4), True)
#     >>> numpy.allclose(q, [1, 0, 0, 0])
#     True
#     >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
#     >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
#     True
#     >>> R = rotation_matrix(0.123, (1, 2, 3))
#     >>> q = quaternion_from_matrix(R, True)
#     >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
#     True
#     >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
#     ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
#     >>> q = quaternion_from_matrix(R)
#     >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
#     True
#     >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
#     ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
#     >>> q = quaternion_from_matrix(R)
#     >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
#     True
#     >>> R = random_rotation_matrix()
#     >>> q = quaternion_from_matrix(R)
#     >>> is_same_transform(R, quaternion_matrix(q))
#     True
#     >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
#     ...                    quaternion_from_matrix(R, isprecise=True))
#     True
#     >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
#     >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
#     ...                    quaternion_from_matrix(R, isprecise=True))
#     True

#     """
#     M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
#     if isprecise:
#         q = np.empty((4, ))
#         t = np.trace(M)
#         if t > M[3, 3]:
#             q[0] = t
#             q[3] = M[1, 0] - M[0, 1]
#             q[2] = M[0, 2] - M[2, 0]
#             q[1] = M[2, 1] - M[1, 2]
#         else:
#             i, j, k = 0, 1, 2
#             if M[1, 1] > M[0, 0]:
#                 i, j, k = 1, 2, 0
#             if M[2, 2] > M[i, i]:
#                 i, j, k = 2, 0, 1
#             t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
#             q[i] = t
#             q[j] = M[i, j] + M[j, i]
#             q[k] = M[k, i] + M[i, k]
#             q[3] = M[k, j] - M[j, k]
#             q = q[[3, 0, 1, 2]]
#         q *= 0.5 / math.sqrt(t * M[3, 3])
#     else:
#         m00 = M[0, 0]
#         m01 = M[0, 1]
#         m02 = M[0, 2]
#         m10 = M[1, 0]
#         m11 = M[1, 1]
#         m12 = M[1, 2]
#         m20 = M[2, 0]
#         m21 = M[2, 1]
#         m22 = M[2, 2]
#         # symmetric matrix K
#         K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
#                          [m01+m10,     m11-m00-m22, 0.0,         0.0],
#                          [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
#                          [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
#         K /= 3.0
#         # quaternion is eigenvector of K that corresponds to largest eigenvalue
#         w, V = np.linalg.eigh(K)
#         q = V[[3, 0, 1, 2], np.argmax(w)]
#     if q[0] < 0.0:
#         np.negative(q, q)
#     return q

def RotMatToQuat(R):
    # https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
    ''' Calculate quaternion corresponding to given rotation matrix

    Parameters
    ----------
    M : array-like
      3x3 rotation matrix

    Returns
    -------
    q : (4,) array
      closest quaternion to input matrix, having positive q[0]

    Notes
    -----
    Method claimed to be robust to numerical errors in M

    Constructs quaternion by calculating maximum eigenvector for matrix
    K (constructed from input `M`).  Although this is not tested, a
    maximum eigenvalue of 1 corresponds to a valid rotation.

    A quaternion q*-1 corresponds to the same rotation as q; thus the
    sign of the reconstructed quaternion is arbitrary, and we return
    quaternions with positive w (q[0]).

    References
    ----------
    * http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    * Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the
      quaternion from a rotation matrix", AIAA Journal of Guidance,
      Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN
      0731-5090

    Examples
    --------
    >>> import numpy as np
    >>> q = mat2quat(np.eye(3)) # Identity rotation
    >>> np.allclose(q, [1, 0, 0, 0])
    True
    >>> q = mat2quat(np.diag([1, -1, -1]))
    >>> np.allclose(q, [0, 1, 0, 0]) # 180 degree rotn around axis 0
    True

    '''
    # Qyx refers to the contribution of the y input vector component to
    # the x output vector component.  Qyx is therefore the same as
    # M[0,1].  The notation is from the Wikipedia article.

    m00 = R[0][0]
    m01 = R[0][1]
    m02 = R[0][2]
    m10 = R[1][0]
    m11 = R[1][1]
    m12 = R[1][2]
    m20 = R[2][0]
    m21 = R[2][1]
    m22 = R[2][2]

    Qxx = m00
    Qyx = m01
    Qzx = m02
    Qxy = m10
    Qyy = m11
    Qzy = m12
    Qxz = m20
    Qyz = m21
    Qzz = m22

    #Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = m00, m10, m20, 
    # Fill only lower half of symmetric matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to w,x,y,z quaternion
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    return q

def QuatToRotMat(q):

    # a = [quatAM[0], quatAM[1], quatAM[2], quatAM[3]]
    q = quatNormalized(q)
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]


    xx = qx ** 2
    xy = qx * qy
    xz = qx * qz
    xw = qx * qw
    yy = qy ** 2
    yz = qy * qz
    yw = qy * qw
    zz = qz ** 2
    zw = qz * qw

    m00 = 1 - 2 * (yy + zz)
    m01 = 2 * (xy - zw)
    m02 = 2 * (xz + yw)
    m10 = 2 * (xy + zw)
    m11 = 1 - 2 * (xx + zz)
    m12 = 2 * (yz - xw)
    m20 = 2 * (xz - yw)
    m21 = 2 * (yz + xw)
    m22 = 1 - 2 * (xx + yy)
    R = np.array([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])
    return R

# def QuatToRotMat(q):
#     q = quatNormalized(q)
#     q1 = q[0]
#     q2 = q[1]
#     q3 = q[2]
#     q4 = q[3]

#     R00 = q1**2 - q2**2 -q3**2 +q4**2
#     R01 = 2*(q1*q2 - q4*q3)
#     R02 = 2*(q1*q3 + q4*q2)
#     R10 = 2*(q1*q2 + q4*q3)
#     R11 = -q1**2 + q2**2 - q3**2 + q4**2
#     R12 = 2*(q2*q3 - q4*q1)
#     R20 = 2*(q3*q1 - q4*q2)
#     R21 = 2*(q3*q2 + q4*q1)
#     R22 = 2*q1**2 - q2**2 + q3**2 + q4**2

#     R = np.mat([[R00, R01, R02],[R10, R11, R12],[R20, R21, R22]])
#     return R

# def RotMatToQuat(R):
#     m00 = R[0][0]
#     m01 = R[0][1]
#     m02 = R[0][2]
#     m10 = R[1][0]
#     m11 = R[1][1]
#     m12 = R[1][2]
#     m20 = R[2][0]
#     m21 = R[2][1]
#     m22 = R[2][2]

#     w = np.sqrt(1 + m00 + m11 + m22)/2.0
#     w4 = 4.0 * w
#     x = (m21 - m12)/w4
#     y = (m02 - m20)/w4
#     z = (m10 - m01)/w4

#     q = [w, x, y, z]
#     q = quatNormalized(q)
#     return q

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

    #q = quatNormalized(q)
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


def qnormalized(q): 
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
    return QuaternionClass(qw, qx, qy, qz)


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


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def AxisAngleToRotMat(axis, angle):
    c = cos(angle)
    s = sin(angle)
    t = 1- c

    axis = axis/LA.norm(axis)
    x = axis[0]
    y = axis[1]
    z = axis[2]

    m00 = c + x*x*t 
    m01 = t*x*y - z*s
    m02 = t*x*z + y*s
    m10 = t*x*y + z*s
    m11 = t*y*y + c
    m12 = t*y*z - x*s
    m20 = t*x*z - y*s
    m21 = t*y*z + x*s
    m22 = t*z*z + c
    R = np.array([[m00, m01, m02],[m10, m11, m12],[m20, m21, m22]])
    return R 








