import numpy as np
import bins.math as bmt
import math
import bins.data as d


class Integrator(object):
    def __init__(self, integration_step):
        self.__integration_step = integration_step
        self.__val = {}
        self.__reset()
        self.__prev = None

    def __reset(self):
        self.__val.update({
            'acc': np.zeros((3, 1)),
            'gyro': np.zeros((3, 1)),
            'interval': 0.0
        })

    def __set_prev(self, acc, gyro, time):
        self.__prev.update({
            'acc': acc,
            'gyro': gyro,
            'time': time
        })

    def __calc(self, acc, gyro, time):
        dt = time - self.__prev['time']
        self.__val['acc'] += (self.__prev['acc'] + acc) * dt * 0.5
        self.__val['gyro'] += (self.__prev['gyro'] + gyro) * dt * 0.5
        self.__val['interval'] += dt

    def receive(self, acc, gyro, time):
        if self.__prev is None:
            self.__prev = {}
            self.__set_prev(acc, gyro, time)
        else:
            self.__calc(acc, gyro, time)
            self.__set_prev(acc, gyro, time)
            if self.__val['interval'] >= self.__integration_step:
                res = self.__val['acc'], self.__val['gyro']
                self.__reset()
                return res


class Compensator(object):
    NUMBER_STEP = 8

    @staticmethod
    def sculling(angles, rates):
        curr = np.zeros((3, 1))
        for angle, rate in zip(angles, rates):
            prev = curr.copy()
            # First approximation
            curr[0, 0] = prev[0, 0] + prev[1, 0] * angle[2, 0] - prev[2, 0] * angle[1, 0] + rate[0, 0]
            curr[1, 0] = prev[1, 0] + prev[2, 0] * angle[0, 0] - prev[0, 0] * angle[2, 0] + rate[1, 0]
            curr[2, 0] = prev[2, 0] + prev[0, 0] * angle[1, 0] - prev[1, 0] * angle[0, 0] + rate[2, 0]
            curr[0, 0] = prev[0, 0] + rate[0, 0]
            curr[1, 0] = prev[1, 0] + rate[1, 0]
            curr[2, 0] = prev[2, 0] + rate[2, 0]
            # Second approximation
            # todo не уверен, надо перепроверить prev - curr
            curr[2, 0] = prev[2, 0] + curr[0, 0] * angle[1, 0] - curr[1, 0] * angle[0, 0] + rate[2, 0]
            curr[1, 0] = prev[1, 0] + curr[2, 0] * angle[0, 0] - curr[0, 0] * angle[2, 0] + rate[1, 0]
            curr[0, 0] = prev[0, 0] + curr[1, 0] * angle[2, 0] - curr[2, 0] * angle[1, 0] + rate[0, 0]
            curr[2, 0] = prev[2, 0] + rate[2, 0]
            curr[1, 0] = prev[1, 0] + rate[1, 0]
            curr[0, 0] = prev[0, 0] + rate[0, 0]
        return curr

    @staticmethod
    def coning(angles):
        def package(val):
            return np.array([[0, -val[2, 0], val[1, 0]],
                             [val[2, 0], 0, -val[0, 0]],
                             [-val[1, 0], val[0, 0], 0]])

        rotate = np.zeros((3, 1))
        p = []
        for angle in angles:
            rotate += angle
            p.append(package(angle))
        # todo - не уверен, поправить
        rotate += 2 / 3 * (np.dot(p[0], angles[1]) + np.dot(p[2], angles[3])) + 0.5 * np.dot(
            p[0] + p[1], angles[2] + angles[3]) + np.dot(p[0] - p[1], angles[2] - angles[3]) / 30
        return rotate

    def __init__(self):
        self.__rates = []
        self.__angles = []
        self.__reset()

    def __append(self, rate, angle):
        self.__rates.append(rate)
        self.__angles.append(angle)
        self.__count += 1

    def __calc(self):
        rate = Compensator.sculling(self.__angles, self.__rates)
        rotate = Compensator.coning(self.__angles)
        return rate, rotate

    def __reset(self):
        self.__count = 0
        self.__rates.clear()
        self.__angles.clear()

    def receive(self, rate, angle):
        self.__append(rate, angle)
        if self.__count >= Compensator.NUMBER_STEP:
            res = self.__calc()
            self.__reset()
            return res


class Navigator(object):
    def __init__(self, navigation, attitude, vertical):
        quat = bmt.attitude2quat(attitude)
        trm = bmt.navigation2transform(navigation, attitude.azimuth)
        self.__or_calc = bmt.create_orientation_calc(quat)
        self.__nav_calc = bmt.create_navigation_calc(trm)
        self.__w = np.array([[bmt.EARTH_ROTATE_RATE * trm[0, 2]],
                             [bmt.EARTH_ROTATE_RATE * trm[1, 2]],
                             [bmt.EARTH_ROTATE_RATE * trm[2, 2]]])
        self.__vertical = vertical

    def receive(self, rate, rotate, step):
        c = self.__or_calc(rotate, self.__w, step)
        b, v, self.__w = self.__nav_calc(rate, c, self.__vertical.altitude, self.__vertical.rate, step)
        return c, b, v


def recalculate(tm_c, tm_b, v):
    def coordinate():
        b0 = math.sqrt(tm_b[2, 0] ** 2 + tm_b[1, 2] ** 2)
        lati, longi = math.atan(tm_b[2, 2] / b0), math.atan(tm_b[2, 1] / tm_b[2, 0])
        az = math.atan(tm_b[0, 2] / tm_b[1, 2])
        return lati, longi, az

    def velocity(az):
        s = math.sin(az)
        c = math.cos(az)
        return v[1] * c + v[0] * s, -v[1] * s + v[0] * c

    def attitude():
        c0 = math.sqrt(tm_c[2, 0] ** 2 + tm_c[2, 2] ** 2)
        return math.atan(tm_c[2, 1] / c0), -math.atan(tm_c[2, 0] / tm_c[2, 2]), math.atan(tm_c[0, 1] / tm_c[1, 1])

    latitude, longitude, azimuth = coordinate()
    north_rate, east_rate = velocity(azimuth)
    pitch, roll, yaw = attitude()
    return (d.Navigation(latitude, longitude, north_rate, east_rate),
            d.Attitude(pitch, roll, yaw, azimuth, yaw - azimuth))


class Bins(object):
    def __init__(self, settings, consumer):
        self.__integrator = Integrator(1 / (settings.out_frequency * Compensator.NUMBER_STEP))
        self.__compensator = Compensator()
        self.__navigator = Navigator(settings.navigation, settings.attitude, settings.vertical)
        self.__consumer = consumer
        self.__time = None

    def receive(self, acc, gyro, time):
        if self.__time is None:
            self.__time = time
        res = self.__integrator.receive(acc, gyro, time)
        if res is not None:
            rate, angle = res
            res = self.__compensator.receive(rate, angle)
            if res is not None:
                rate, rotate = res
                c, b, v = self.__navigator.receive(rate, rotate, time - self.__time)
                navigation, attitude = recalculate(c, b, v)
                self.__time = time
                self.__consumer.send(navigation, attitude, time)

    def stop(self):
        self.__consumer.stop()
