import math
import pyquaternion as qt
import numpy as np

EARTH_ROTATE_RATE = 7.292115e-5
SEMIMAJOR = 6378137.0
SEMIMINOR = 6356752.3
ECCENTRICITY = math.sqrt(1 - SEMIMINOR ** 2 / SEMIMAJOR ** 2)  # todo точно?


def create_orientation_calc(st_quat):
    def rotate_quat(rotate):
        df2 = math.sqrt((rotate ** 2).sum()) ** 2
        df4 = df2 ** 2
        r = 0.5 - df2 / 42 - df4 / 3840
        return qt.Quaternion(1 - df2 / 8 + df4 / 384, r * rotate[0, 0], r * rotate[1, 0], r * rotate[2, 0])

    def rate_quat(rate, step):
        w = math.sqrt((rate ** 2).sum())
        val = w * step / 2
        var = -rate / w * math.sin(val)
        return qt.Quaternion(math.cos(val), var[0, 0], var[1, 0], var[2, 0])

    def calc(rotate, rate, step):
        nonlocal quat
        tmp_quat = quat * rotate_quat(rotate)
        quat = rate_quat(rate, step) * tmp_quat
        return quat.rotation_matrix

    quat = st_quat
    return calc


def create_navigation_calc(transform_matrix):
    def relate_angular_rate(v, b, alt):
        e = ECCENTRICITY ** 2
        rx = (1 - e * b[2, 2] ** 2 / 2 + e * b[0, 2] ** 2 - alt / SEMIMAJOR) / SEMIMAJOR
        ry = (1 - e * b[2, 2] ** 2 / 2 + e * b[1, 2] ** 2 - alt / SEMIMAJOR) / SEMIMAJOR
        mu_x = -v[1] * ry - v[0] * e * b[0, 2] * b[1, 2] / SEMIMAJOR
        mu_y = v[0] * rx + v[1] * e * b[0, 2] * b[1, 2] / SEMIMAJOR
        return mu_x, mu_y

    def absolute_angular_rate(mu, u):
        return np.array([[mu[0] + u[0]], [mu[1] + u[1]], [u[2]]])

    def relate_earth_rate(b):
        return EARTH_ROTATE_RATE * b[0, 2], EARTH_ROTATE_RATE * b[1, 2], EARTH_ROTATE_RATE * b[2, 2]

    def nav2earth(b, mu, step):
        new = np.zeros((3, 3))
        new[0, 1] = b[0, 1] - mu[1] * b[2, 1] * step
        new[1, 1] = b[1, 1] + mu[0] * b[2, 1] * step
        new[2, 1] = b[2, 1] + (mu[1] * b[0, 1] - mu[0] * b[1, 1]) * step
        new[0, 2] = b[0, 2] - mu[1] * b[2, 2] * step
        new[1, 2] = b[1, 2] + mu[0] * b[2, 2] * step
        new[2, 2] = b[2, 2] + (mu[1] * b[0, 2] + mu[0] * b[1, 2]) * step
        new[2, 0] = b[0, 1] * b[1, 2] - b[1, 1] * b[0, 2]
        return new

    def recalc_rate(v, tm):
        res = np.dot(tm, v)
        return res[0, 0], res[1, 0]

    def recalc_earth(v, u, vz, mu, step):
        res_x = v[1] * 2 * u[2] - vz * (mu[1] + 2 * u[1])
        res_y = v[0] * 2 * u[2] - vz * (mu[0] + 2 * u[0])
        return res_x * step, res_y * step

    def add(a, b):
        a0, a1 = a
        b0, b1 = b
        return a0 + b0, a1 + b1

    def calc(rate, orientation, altitude, vertical_rate, step):
        nonlocal ground, tm_local, apartment, earth
        mu = relate_angular_rate(ground, tm_local, altitude)
        u = relate_earth_rate(tm_local)
        angular = absolute_angular_rate(mu, u)
        tm_local = nav2earth(tm_local, mu, step)
        apartment = add(apartment, recalc_rate(rate, orientation))
        earth = add(earth, recalc_earth(ground, u, vertical_rate, mu, step))
        ground = apartment + earth
        return tm_local, ground, angular

    tm_local = transform_matrix
    ground = 0, 0
    apartment = 0, 0
    earth = 0, 0
    return calc


def navigation2transform(nav, azimuth):
    latitude = nav.latitude
    longitude = nav.longitude
    b = np.zeros((3, 3), float)
    sin = math.sin
    cos = math.cos
    b[0, 0] = -sin(latitude) * cos(longitude) * sin(azimuth) - sin(longitude) * cos(azimuth)
    b[0, 1] = sin(latitude) * sin(longitude) * sin(azimuth) + cos(longitude) * cos(azimuth)
    b[0, 2] = cos(latitude) * sin(azimuth)
    b[1, 0] = -sin(latitude) * cos(longitude) * cos(azimuth) + sin(longitude) * sin(azimuth)
    b[1, 1] = -sin(latitude) * sin(longitude) * cos(azimuth) - cos(longitude) * sin(azimuth)
    b[1, 2] = cos(latitude) * cos(azimuth)
    b[2, 0] = cos(latitude) * cos(longitude)
    b[2, 1] = cos(latitude) * sin(longitude)
    b[2, 2] = sin(latitude)
    return b


def attitude2quat(attitude):
    #todo точно?
    pitch, roll, yaw, _, _ = attitude
    sin = math.sin
    cos = math.cos
    p = qt.Quaternion(cos(yaw / 2), 0.0, sin(yaw / 2), 0.0)
    q = qt.Quaternion(cos(pitch / 2), 0.0, 0.0, sin(pitch / 2))
    r = qt.Quaternion(cos(roll / 2), sin(roll / 2), 0.0, 0.0)
    return (p * q) * r


def grad2rad(angle):
    return angle * math.pi / 180


def rad2grad(angle):
    return angle * 180 / math.pi


# Расчет ведется по формуле взятой из Гравиметрия - Грушинский Н.П.
# Это эмпирическая формула, справедливая для небольших отклонений от поверхности Земли
# todo - возможно следует найти более качественные модели расчета
def acceleration_gravity(latitude, altitude):
    return 9.780318 * (1 + 0.005302 * math.sin(latitude) ** 2
                       - 0.000006 * math.sin(2 * latitude) ** 2) - 0.000003086 * altitude
