import numpy as np
import pyquaternion as qt
import math as mt
from .model import local_rate, earth_rate
from collections import namedtuple

Navigation = namedtuple('Navigation', 'earth_dcm nav_dcm rate time')


def accumulate(data):
    """
    Производит интегрирование методом трапеций.
    :param data: - фрейм данных, матрица (?,7).
        Элемент (?,0/1/2) - x, y, z оси акселерометра.
        Элемент (?,3/4/5) - x, y, z оси гироскопа.
        Элемент (?,6) - момент времени (timestamp) (ед. измерения - секунда).
    :return:
        raw_data: - фрейм данныхб вектор (7,)
        Элемент (0:3) - вектор приращений скорости.
        Элемент (3:6) - вектор приращений угла.
        Элемент (7) - момент времени (отсечка в момент сброса интегратора).
    """
    res = np.einsum('ji,j->i', data[1:, :] + data[:-1, :], data[1:, -1] - data[:-1, -1]) / 2
    res[-1] = data[-1, -1]
    return res


def coning_sculling(data):
    """
    Производит coning-sculling коррекцию.
    :param data: фрейм данных, матрица (4, 7)
        accel - срез (:, 0:3) приращений линейной скорости.
        gyro - срез (:, 3:6) приращений угла поворота.
        time - срез (:, 7)
    :return:
        raw_data - фрейм данных, вектор (7,)
        Элемент (0:3) - вектор приращений скорости.
        Элемент (3:6) - вектор конечного поворота.
        Элемент (7) - момент времени (отсечка в момент сброса интегратора).
    """
    accel = data[:, 0:3]
    gyro = data[:, 3:6]
    res = np.zeros((7,))

    # Sculling
    res[0:3] = np.sum((np.sum(accel, axis=0) - np.cross(gyro, accel)), axis=0) / 4

    # Coning
    res[3:6] = (np.einsum('ji->i', gyro)
                + 2 / 3 * (np.cross(gyro[0], gyro[1]) + np.cross(gyro[2], gyro[3]))
                + 8 / 15 * (np.cross(gyro[0], gyro[2]) + np.cross(gyro[1], gyro[3]))
                + 7 / 15 * (np.cross(gyro[0], gyro[3]) + np.cross(gyro[1], gyro[2])))

    res[-1] = data[-1, -1]
    return res


def recalc(data, old, alt, dt):
    # Angular rate
    omega = local_rate(old.earth_dcm, old.rate, alt)
    u = earth_rate(old.earth_dcm)
    w = omega + u

    # Puasson equation
    e_dcm = np.zeros((3, 3))
    e_dcm[0, 1] = old.earth_dcm[0, 1] - omega[1] * old.earth_dcm[2, 1] * dt
    e_dcm[1, 1] = old.earth_dcm[1, 1] + omega[0] * old.earth_dcm[2, 1] * dt
    e_dcm[2, 1] = old.earth_dcm[2, 1] + (omega[1] * old.earth_dcm[0, 1] - omega[0] * old.earth_dcm[1, 1]) * dt
    e_dcm[0, 2] = old.earth_dcm[0, 2] - omega[1] * old.earth_dcm[2, 2] * dt
    e_dcm[1, 2] = old.earth_dcm[1, 2] + omega[0] * old.earth_dcm[2, 2] * dt
    e_dcm[2, 2] = old.earth_dcm[2, 2] + (omega[1] * old.earth_dcm[0, 2] + omega[0] * old.earth_dcm[1, 2]) * dt
    e_dcm[2, 0] = old.earth_dcm[0, 1] * old.earth_dcm[1, 2] - old.earth_dcm[1, 1] * old.earth_dcm[0, 2]

    # Rate recalc
    dv = np.dot(old.nav_dcm, data[0:3])
    rate = old.rate + dv + np.array([old.rate[1] * 2 * u[2] - old.rate[2] * (omega[1] + 2 * u[1]),
                                     old.rate[0] * 2 * u[2] - old.rate[2] * (omega[0] + 2 * u[0]),
                                     0]) * dt

    # Fast motion
    df2 = (data[3:6] ** 2).sum()
    df4 = df2 ** 2
    r = 0.5 - df2 / 42 - df4 / 3840
    fast = qt.Quaternion(1 - df2 / 8 + df4 / 384, r * data[3], r * data[4], r * data[5])

    # Slow motion
    om = (w ** 2).sum() ** 0.5
    val = om * dt / 2
    var = -w / om * mt.sin(val)
    slow = qt.Quaternion(mt.cos(val), var[0], var[1], var[2])

    # Orientation recalc
    n_dcm = (slow * (qt.Quaternion(matrix=old.nav_dcm) * fast)).rotation_matrix

    return Navigation(earth_dcm=e_dcm, nav_dcm=n_dcm, rate=rate, time=data[-1])
