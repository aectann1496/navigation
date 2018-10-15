from collections import namedtuple


Navigation = namedtuple('Navigation', 'latitude longitude north_rate east_rate')
Attitude = namedtuple('Attitude', 'pitch roll yaw azimuth heading')
Vertical = namedtuple('Vertical', 'altitude rate')
Settings = namedtuple('Settings', 'navigation attitude vertical out_frequency')
