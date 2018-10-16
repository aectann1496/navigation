from abc import ABCMeta, abstractmethod
from collections import namedtuple
import matplotlib.pyplot as plt
import numpy as np


class AbstractConsumer(metaclass=ABCMeta):
    @abstractmethod
    def send(self, navigation, attitude, time):
        raise NotImplementedError()

    @abstractmethod
    def stop(self):
        pass


class TerminalAbstractConsumer(AbstractConsumer, metaclass=ABCMeta):
    def __init__(self):
        self.__data = []

    def send(self, navigation, attitude, time):
        self.__data.append((navigation, attitude, time))

    def stop(self):
        self.processing(self.__data)
        self.__data = []

    @abstractmethod
    def processing(self, elements):
        raise NotImplementedError()


class MultiplierConsumer(AbstractConsumer):
    def __init__(self, *args):
        self.__consumers = args

    def send(self, navigation, attitude, time):
        for consumer in self.__consumers:
            consumer.send(navigation, attitude, time)

    def stop(self):
        for consumer in self.__consumers:
            consumer.stop()


class PrintConsumer(AbstractConsumer):
    def __init__(self, file_name=None):
        if file_name is not None:
            self.__file = open(file_name, "w")
        else:
            self.__file = None

    def send(self, navigation, attitude, time):
        print(navigation, attitude, time, sep=';', file=self.__file)

    def stop(self):
        if self.__file is not None:
            self.__file.close()


class PlotConsumer(TerminalAbstractConsumer):
    NavigationData = namedtuple('NavigationData', ['time',
                                                   'latitude', 'longitude',
                                                   'north', 'east',
                                                   'pitch', 'roll', 'yaw', 'azimuth', 'heading'])
    ROWS = 3
    COLS = 4

    def __convert_data(self, data):
        nav, att, t = data
        return PlotConsumer.NavigationData(t, nav.latitude, nav.longitude, nav.north_rate, nav.east_rate,
                                           att.pitch, att.roll, att.yaw, att.azimuth, att.heading)

    def __fill(self, n, title, x_label, y_label, nav_data, time):
        plt.subplot(PlotConsumer.ROWS, PlotConsumer.COLS, n)
        plt.plot(time, np.array([d[n] for d in nav_data]))
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.grid()

    def processing(self, data):
        nav_data = list(map(self.__convert_data, data))
        time = np.array([d[0] for d in nav_data])
        fig, _ = plt.subplots(PlotConsumer.ROWS, PlotConsumer.COLS)
        fig.tight_layout()
        self.__fill(1, 'Широта', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(2, 'Долгота', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(3, 'Северная составляющая скорости', 'время, с.', 'скорость, м/с', nav_data, time)
        self.__fill(4, 'Восточная составляющая скорости', 'время, с.', 'скорость, м/с', nav_data, time)
        self.__fill(5, 'pitch', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(6, 'roll', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(7, 'yaw', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(8, 'azimuth', 'время, с.', 'угол, рад', nav_data, time)
        self.__fill(9, 'heading', 'время, с.', 'угол, рад', nav_data, time)
        plt.show()
