import sins.data
import sins.sim
import sins.plotting

att = sins.data.attitude(pitch=0, roll=0, yaw=0)
pos = sins.data.position(latitude=0, longitude=0, wander=0, altitude=100)
vel = sins.data.velocity(north=0, east=0, up=0)

data = sins.sim.stationary_data(pos, att, 20, 20*2000)
raw = sins.sim.integrate(data, pos, att, vel, 1/2000*40)

import numpy as np
import sins.dcm
import math as mt
res = np.zeros((len(raw), 9))
for i, elem in enumerate(raw):
    llw = sins.dcm.to_llw(elem.earth_dcm)
    pry = sins.dcm.to_pry(elem.nav_dcm)
    v = elem.rate
    rate = (v[1] * mt.cos(llw[2]) + v[0] * mt.sin(llw[2]),
                -v[1] * mt.sin(llw[2]) + v[0] * mt.cos(llw[2]))

    res[i, 0:3] = np.asarray(llw)
    res[i, 3:6] = np.asarray(pry)
    res[i, 6:8] = np.asarray(rate)
    res[i, -1] = elem.time

sins.plotting.Plotter()\
    .figure('Положение', (2, 2))\
        .subplot('Широта', 'Время, с.', '', res[:, -1], res[:, 0])\
        .subplot('Долгота', 'Время, с.', '', res[:, -1], res[:, 1])\
        .subplot('Азимут', 'Время, с.', '', res[:, -1], res[:, 2])\
    .figure('Ориентация', (2, 2))\
        .subplot('Крен', 'Время, с.', '', res[:, -1], res[:, 3])\
        .subplot('Тангаж', 'Время, с.', '', res[:, -1], res[:, 4])\
        .subplot('Рысканье', 'Время, с.', '', res[:, -1], res[:, 5])\
    .figure('Скорости', (2, 1))\
        .subplot('Северная', 'Время, с.', '', res[:, -1], res[:, 6])\
        .subplot('Восточная', 'Время, с.', '', res[:, -1], res[:, 7])\
    .plot()
