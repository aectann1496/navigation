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
    accum = []
    for i in range(10, data.shape[0], 10):
        accum.append(_integrate.accumulate(data[i-10:i,:]))
    accum = np.asarray(accum)

    raw = []
    for i in range(4, accum.shape[0], 4):
        raw.append(_integrate.coning_sculling(accum[i-4:i, :]))
    raw = np.asarray(raw)

    lat, lon, wan, alt = pos
    nav = _integrate.Navigation(earth_dcm=dcm.from_llw(lat, lon, wan).transpose(),
                                nav_dcm=dcm.from_pry(*att),
                                rate=np.asarray(vel),
                                time=data[0, -1])
    res = [nav]

    for i in range(0, raw.shape[0]):
        nav = _integrate.recalc(raw[i, :], nav, alt, raw[i, -1] - nav.time)
        res.append(nav)

    return res
