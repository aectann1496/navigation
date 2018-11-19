import numpy as np
import pyquaternion as qt
import math as mt
from .model import local_rate, earth_rate


def accumulate(data):
    """
    Производит интегрирование методом трапеций.
    :param data: - фрейм данных, матрица (?,7).
        Элемент (?,0/1/2) - x, y, z оси акселерометра.
        Элемент (?,3/4/5) - x, y, z оси гироскопа.
        Элемент (?,6) - момент времени (timestamp) (ед. измерения - секунда).
    :return:
        accel - вектор (3,) приращений линейной скорости.
        gyro - вектор (3,) приращений угла поворота.
    """
    return np.einsum('ji,j->i', data[1:, :-1] + data[:-1, :-1], data[1:, -1] - data[:-1, -1]) / 2


def coning_sculling(data):
    """
    Производит coning-sculling коррекцию.
    :param data: матрица (4, 6)
        accel - срез (4, 0:3) приращений линейной скорости.
        gyro - срез (4, 3:6) приращений угла поворота.
    :return:
        dv - вектор (3,) приращений линейной скорости.
        rvc - вектор (3,) конечного поворота.
    """
    accel = data[:, :-3]
    gyro = data[:, 3:]

    dv = np.sum((np.sum(accel, axis=0) - np.cross(gyro, accel)), axis=0) / 4
    rvc = (np.einsum('ji->i', gyro)
           + 2 / 3 * (np.cross(gyro[0], gyro[1]) + np.cross(gyro[2], gyro[3]))
           + 8 / 15 * (np.cross(gyro[0], gyro[2]) + np.cross(gyro[1], gyro[3]))
           + 7 / 15 * (np.cross(gyro[0], gyro[3]) + np.cross(gyro[1], gyro[2])))
    return dv, rvc


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


def _puasson(dcm, rate, dt):
    new = np.zeros((3, 3))
    new[0, 1] = dcm[0, 1] - rate[1] * dcm[2, 1] * dt
    new[1, 1] = dcm[1, 1] + rate[0] * dcm[2, 1] * dt
    new[2, 1] = dcm[2, 1] + (rate[1] * dcm[0, 1] - rate[0] * dcm[1, 1]) * dt
    new[0, 2] = dcm[0, 2] - rate[1] * dcm[2, 2] * dt
    new[1, 2] = dcm[1, 2] + rate[0] * dcm[2, 2] * dt
    new[2, 2] = dcm[2, 2] + (rate[1] * dcm[0, 2] + rate[0] * dcm[1, 2]) * dt
    new[2, 0] = dcm[0, 1] * dcm[1, 2] - dcm[1, 1] * dcm[0, 2]
    return new


def integration(dcm, v, dv, dt, alt):
    omega = local_rate(dcm, v, alt)
    u = earth_rate(dcm)
    w = omega + u

    dcm = _puasson(dcm, omega, dt)

    v = v + dv + np.array([v[1] * 2 * u[2] - v[2] * (omega[1] + 2 * u[1]),
                           v[0] * 2 * u[2] - v[2] * (omega[0] + 2 * u[0]),
                           0]) * dt

    return dcm, v, w
