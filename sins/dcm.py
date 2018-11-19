import numpy as np
from math import sin, cos, atan, sqrt


def from_pry(p, r, y):
    """

    :param p:
    :param r:
    :param y:
    :return:
    """
    sp = sin(p)
    cp = cos(p)
    sr = sin(r)
    cr = cos(r)
    sy = sin(y)
    cy = cos(y)

    dcm = np.empty((3, 3))
    dcm[0, 0] = cy * cr - sy * sp * sr
    dcm[0, 1] = -sy * cp
    dcm[0, 2] = cy * sr + sy * sp * cr
    dcm[1, 0] = sy * cr + cy * sp * sr
    dcm[1, 1] = cy * cp
    dcm[1, 2] = sy * sr - cy * sp * cr
    dcm[2, 0] = -cp * sr
    dcm[2, 1] = sp
    dcm[2, 2] = cp * cr

    return dcm


def to_pry(dcm):
    """

    :param dcm:
    :return:
    """
    p = atan(dcm[2, 1] / sqrt(dcm[0, 1] ** 2 + dcm[1, 1] ** 2))
    r = -atan(dcm[0, 1] / dcm[1, 1])
    y = -atan(dcm[2, 0] / dcm[2, 2])

    return p, r, y


def from_llw(lat, lon, wan=0):
    """

    :param lat:
    :param lon:
    :param wan:
    :return:
    """
    sf = sin(lat)
    cf = cos(lat)
    sl = sin(lon)
    cl = cos(lon)
    sa = sin(wan)
    ca = cos(wan)

    dcm = np.empty((3, 3))
    dcm[0, 0] = -sl * ca - cl * sf * sa
    dcm[0, 1] = sl * sa - cl * sf * ca
    dcm[0, 2] = cl * cf
    dcm[1, 0] = cl * ca - sl * sf * sa
    dcm[1, 1] = -cl * sa - sl * sf * ca
    dcm[1, 2] = sl * cf
    dcm[2, 0] = cf * sa
    dcm[2, 1] = cf * ca
    dcm[2, 2] = sf

    return dcm


def to_llw(dcm):
    """

    fixme: неверифицированно!
    :param dcm:
    :return:
    """
    lat = atan(dcm[2, 2] / sqrt(dcm[2, 0] ** 2 + dcm[1, 2] ** 2))
    lon = atan(dcm[2, 1] / dcm[2, 0])
    wan = atan(dcm[0, 2] / dcm[1, 2])

    return lat, lon, wan
