import sins.data
import sins.sim
import sins.plotting

att = sins.data.attitude(pitch=0, roll=0, yaw=0)
pos = sins.data.position(latitude=0, longitude=0, wander=0, altitude=100)
vel = sins.data.velocity(north=0, east=0, up=0)

data = sins.sim.stationary_data(pos, att, 20, 20*2000)
res = sins.sim.integrate(data, pos, att, vel)

sins.plotting.Plotter()\
    .figure('Положение')\
        .subplot('Широта', 'Время, с.', '', res[:, -1], res[:, 0])\
        .subplot('Долгота', 'Время, с.', '', res[:, -1], res[:, 1])\
        .subplot('Азимут', 'Время, с.', '', res[:, -1], res[:, 2])\
    .figure('Ориентация')\
        .subplot('Крен', 'Время, с.', '', res[:, -1], res[:, 3])\
        .subplot('Тангаж', 'Время, с.', '', res[:, -1], res[:, 4])\
        .subplot('Рысканье', 'Время, с.', '', res[:, -1], res[:, 5])\
    .figure('Скорости')\
        .subplot('Северная', 'Время, с.', '', res[:, -1], res[:, 6])\
        .subplot('Восточная', 'Время, с.', '', res[:, -1], res[:, 7])\
    .plot()
