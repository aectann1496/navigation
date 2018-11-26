import sins.data
import sins.sim
import sins.plotting

att = sins.data.attitude(pitch=0, roll=0, yaw=0)
pos = sins.data.position(latitude=0, longitude=0, wander=0, altitude=100)
vel = sins.data.velocity(north=0, east=0, up=0)

data = sins.sim.stationary_data(pos, att, 20, 20*2000)
res = sins.sim.integrate(data, pos, att, vel, 1/2000*40)

import numpy as np
t = np.array([i*1/2000*40 for i in range(len(res))])
sins.plotting.Plotter()\
    .figure('Положение', (2, 2))\
        .subplot('Широта', 'Время, с.', '', t, [e[0] for e, _, _ in res])\
        .subplot('Долгота', 'Время, с.', '', t, [e[1] for e, _, _ in res])\
        .subplot('Азимут', 'Время, с.', '', t, [e[2] for e, _, _ in res])\
    .figure('Ориентация', (2, 2))\
        .subplot('Крен', 'Время, с.', '', t, [e[0] for _, e, _ in res])\
        .subplot('Тангаж', 'Время, с.', '', t, [e[1] for _, e, _ in res])\
        .subplot('Рысканье', 'Время, с.', '', t, [e[2] for _, e, _ in res])\
    .figure('Скорости', (2, 1))\
        .subplot('Северная', 'Время, с.', '', t, [e[0] for _, _, e in res])\
        .subplot('Восточная', 'Время, с.', '', t, [e[0] for _, _, e in res])\
    .plot()
