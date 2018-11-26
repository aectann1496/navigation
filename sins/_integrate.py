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


def orientation(quat, v, rvc, dt):
    df2 = (rvc ** 2).sum()
    df4 = df2 ** 2
    r = 0.5 - df2 / 42 - df4 / 3840
    fast = qt.Quaternion(1 - df2 / 8 + df4 / 384, r * rvc[0], r * rvc[1], r * rvc[2])

    w = (v ** 2).sum() ** 0.5
    val = w * dt / 2
    var = -v / w * mt.sin(val)
    slow = qt.Quaternion(mt.cos(val), var[0], var[1], var[2])

    return slow * (quat * fast)


def integration(dcm, v, dv, dt, alt):
    omega = local_rate(dcm, v, alt)
    u = earth_rate(dcm)
    w = omega + u

    # Puasson evaluation
    new_dcm = np.zeros((3, 3))
    new_dcm[0, 1] = dcm[0, 1] - omega[1] * dcm[2, 1] * dt
    new_dcm[1, 1] = dcm[1, 1] + omega[0] * dcm[2, 1] * dt
    new_dcm[2, 1] = dcm[2, 1] + (omega[1] * dcm[0, 1] - omega[0] * dcm[1, 1]) * dt
    new_dcm[0, 2] = dcm[0, 2] - omega[1] * dcm[2, 2] * dt
    new_dcm[1, 2] = dcm[1, 2] + omega[0] * dcm[2, 2] * dt
    new_dcm[2, 2] = dcm[2, 2] + (omega[1] * dcm[0, 2] + omega[0] * dcm[1, 2]) * dt
    new_dcm[2, 0] = dcm[0, 1] * dcm[1, 2] - dcm[1, 1] * dcm[0, 2]

    v = v + dv + np.array([v[1] * 2 * u[2] - v[2] * (omega[1] + 2 * u[1]),
                           v[0] * 2 * u[2] - v[2] * (omega[0] + 2 * u[0]),
                           0]) * dt

    return new_dcm, v, w


def recalc(data, old, alt, dt):
    e_dcm, rate, w = integration(old.earth_dcm, old.rate, np.dot(old.nav_dcm, data[0:3]), dt, alt)
    n_dcm = orientation(qt.Quaternion(matrix=old.nav_dcm), w, data[3:6], dt).rotation_matrix
    return Navigation(earth_dcm=e_dcm, nav_dcm=n_dcm, rate=rate, time=data[-1])
