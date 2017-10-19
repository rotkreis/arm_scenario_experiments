"""This module implements useful, often used, functions including conversion from ROS Points and Quaternions to numpy arrays, rotations and change of coordinates

# WARNING #  Unlike the famous transformations.py which denotes quaternions as arrays [w, x, y, z],
# WARNING #  tf.transformations denotes quaternions as arrays [x, y, z, w] !!!!!
"""

from tf import transformations

np = transformations.numpy

quat_mul = transformations.quaternion_multiply
quat_conj = transformations.quaternion_conjugate


def point2array(point):
    return np.array([point.x, point.y, point.z])


def quat2array(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])


def quat_rotate(point, quaternion):
    p = np.concatenate((point, [0]))
    res_inter = quat_mul(quaternion, p)
    return quat_mul(res_inter, quat_conj(quaternion))[0:3]


def change_CS(point, translation, quaternion):
    """ Get the coordinates of a point in frame B from its coordinates in frmae A.
        The translation must be the coordinates expressed in frame A of the origin of frame B
        The rotation must be the active rotation that takes the points (A,B,C) of coordinates {(1,0,0) (0,1,0) (0,0,1)} in frame A to the points (A',B',C') of coordinates {(1,0,0) (0,1,0) (0,0,1)} in frame B, when the two frames origines are superimposed (no translation)"""
    return quat_rotate((point - translation), quat_conj(quaternion))


def change_CS_reverse(point, translation, quaternion):
    return quat_rotate(-(point - translation), quat_conj(quaternion))
