import numpy as np


def attitude(pitch, roll, yaw):
    return np.deg2rad(pitch), np.deg2rad(roll), np.deg2rad(yaw)


def position(latitude, longitude, wander, altitude):
    return np.deg2rad(latitude), np.deg2rad(longitude), np.deg2rad(wander), altitude


def velocity(north, east, up):
    return east, north, up
