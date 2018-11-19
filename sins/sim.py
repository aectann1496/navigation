import numpy as np
import math as mt
from . import model
from . import dcm
from . import _integrate
import pyquaternion as qt


def stationary_data(pos, att, t, size):
    lat, _, _, alt = pos
    p, r, y = att

    g = np.array([0, 0, -model.gravity(lat, alt)])
    u = np.array([0, model.RATE * mt.cos(lat), model.RATE * mt.sin(lat)])
    ll = dcm.from_pry(p, r, y)

    accel = np.dot(ll, g)
    gyro = np.dot(ll, u)

    res = np.empty((size, 7))
    res[:, :-4] = np.asarray([accel for i in range(size)])
    res[:, 3:-1] = np.asarray([gyro for i in range(size)])
    res[:, -1] = np.linspace(0, t, size)

    return res


def integrate(data, pos, att, vel, dt):
    acc = []
    for i in range(10, data.shape[0], 10):
        acc.append(_integrate.accumulate(data[i-10:i,:]))
    acc = np.asarray(acc)

    dv = []
    rvc = []
    for i in range(4, acc.shape[0], 4):
        res = _integrate.coning_sculling(acc[i-4:i, :])
        dv.append(res[0])
        rvc.append(res[1])
    dv = np.asarray(dv)
    rvc = np.asarray(rvc)

    quat = qt.Quaternion(matrix=dcm.from_pry(*att))
    lat, lon, wan, alt = pos
    b = dcm.from_llw(lat, lon, wan).transpose()
    v = np.asarray(vel)
    result = []
    for i in range(0, dv.shape[0]):
        b, v, w = _integrate.integration(b, v, dv[i,:], dt, alt)
        quat = _integrate.orientation(quat, w, rvc[i,:], dt)
        llw = dcm.to_llw(b)
        pry = dcm.to_pry(quat.rotation_matrix)
        rate = (v[1] * mt.cos(llw[2]) + v[0] * mt.sin(llw[2]),
                -v[1] * mt.sin(llw[2]) + v[0] * mt.cos(llw[2]))
        result.append((llw, pry, rate))

    return result
