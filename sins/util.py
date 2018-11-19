import numpy as np


def check_array(arr, shape, name=None):
    if name is None:
        name = 'Array'

    if not isinstance(arr, np.ndarray):
        raise Exception('%s isn`t numpy.ndarray'.format(name))
    if shape is not None and arr.shape != shape:
        raise Exception('Shape of %s isn`t equal %s'.format(name, shape))
