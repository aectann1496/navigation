from bins.math import acceleration_gravity, EARTH_ROTATE_RATE
import numpy as np
import math


class ImmovableModelSupplier(object):

    def __init__(self, navigation, vertical, dt, time, bins):
        self.__gravity = acceleration_gravity(navigation.latitude, vertical.altitude)
        self.__lati = navigation.latitude
        self.__dt = dt
        self.__time = time
        self.__bins = bins

    def acc(self):
        return np.array([[0], [0], [-9.81]])

    def gyro(self):
        return np.array([[0], [EARTH_ROTATE_RATE * math.cos(self.__lati)], [EARTH_ROTATE_RATE * math.sin(self.__lati)]])

    def run(self):
        t = 0
        while t <= self.__time:
            self.__bins.receive(self.acc(), self.gyro(), t)
            t += self.__dt
        self.__bins.stop()
